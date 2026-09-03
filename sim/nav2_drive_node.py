#!/usr/bin/env python3
"""Nav2 の閉ループ検証。目標を順に送り、位置推定と航法を同時に記録する。

`drive_node.py` (開ループ) との決定的な違いは、**制御が位置推定の出力で回る**
ことである。開ループでは位置推定は受け身の観測者だったが、ここでは推定が
ずれれば経路も追従も costmap もずれる。したがってここで初めて測れるのは

  - 大域位置推定の**跳び** (map->odom の不連続) が制御に何を起こすか
  - 推定誤差が実際のクリアランス (壁・動的障害物との距離) に何 m 化けるか
  - kidnap のあと、航法スタックごと復帰できるか

記録 (CSV, 20 Hz):
  t, gt_*, est_* (TF map->base_link), pos_err, yaw_err, dr_*, odom_err,
  wall_clr, obs_clr, cmd_v, cmd_w, goal_idx, tf_jump_m, tf_jump_deg, kidnapped

イベント (JSON): 目標の送信・成否と時刻、Nav2 の起動待ち時間。
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from tf2_ros import Buffer, TransformListener

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from make_env import clearance_map          # noqa: E402  (壁までの距離を同じ定義で)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def load_pgm_clearance(yaml_path):
    """地図 YAML から占有格子を読み、壁までの距離場を返す。"""
    res, ox, oy, image = 0.05, 0.0, 0.0, None
    for line in open(yaml_path):
        if line.startswith('image:'):
            image = line.split(':', 1)[1].strip()
        elif line.startswith('resolution:'):
            res = float(line.split(':', 1)[1])
        elif line.startswith('origin:'):
            v = line.split('[', 1)[1].split(']')[0].split(',')
            ox, oy = float(v[0]), float(v[1])
    path = os.path.join(os.path.dirname(yaml_path), image)
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5'
        w, h = map(int, f.readline().split())
        f.readline()
        img = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    return clearance_map(img, res), ox, oy, res, h


class Nav2Driver(Node):
    def __init__(self, a, goals):
        super().__init__('ofl_nav2_driver')
        self.a = a
        self.goals = goals
        self.gi = 0
        self.laps = 0
        self.rows = []
        self.events = []
        self.t0 = None
        self.done = False
        self.kidnapped = False
        self.reinitialized = False
        self.goal_active = False
        self.goal_handle = None
        self.goal_t0 = None
        self.nav_ready_t = None

        self.wall0 = None            # 実時間倍率 (RTF) の算出用
        self.prev_err = None         # 推定誤差ベクトル (跳びの算出用)
        self.gt = self.gt0 = self.odom = self.odom0 = None
        self.cmd = (0.0, 0.0)
        self.obs = []
        self.prev_mo = None

        self.clr, self.ox, self.oy, self.res, self.mh = load_pgm_clearance(a.map)
        self.dyn = json.load(open(a.dynamic)) if a.dynamic and os.path.exists(a.dynamic) else []

        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, 'ground_truth', self.on_gt, qos)
        self.create_subscription(Odometry, 'odom', self.on_odom, qos)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.create_subscription(PoseArray, '/dyn_obstacles', self.on_obs, 10)
        self.stop_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # 大域 costmap を最後に 1 枚保存する。誤ロックの最中に取り込んだスキャンは
        # **誤った場所へ壁として焼き付き、以後クリアされない**ので、位置推定の
        # 失敗が航法に残す傷はここにしか現れない。
        self.costmap = None
        if a.dump_costmap:
            self.create_subscription(
                OccupancyGrid, '/global_costmap/costmap', self.on_costmap,
                QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                           reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        from gazebo_msgs.srv import SetEntityState
        self.set_state = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        # --reinit-at 用: AMCL の大域位置推定 (パーティクルを自由空間全域へ
        # 一様に撒き直す)。「失敗を完璧に検知できた」というオラクルを仮定して
        # 一様撒き直しという復帰手段そのものの実力を測るための口である
        from std_srvs.srv import Empty
        self.reinit = self.create_client(Empty, '/reinitialize_global_localization')
        self.create_timer(0.05, self.step)

    # ---------- 入力 ----------
    def on_gt(self, m):
        self.gt = (m.pose.pose.position.x, m.pose.pose.position.y,
                   yaw_of(m.pose.pose.orientation))

    def on_odom(self, m):
        self.odom = (m.pose.pose.position.x, m.pose.pose.position.y,
                     yaw_of(m.pose.pose.orientation))
        if self.odom0 is None and self.gt is not None:
            self.odom0, self.gt0 = self.odom, self.gt

    def on_cmd(self, m):
        self.cmd = (m.linear.x, m.angular.z)

    def on_costmap(self, m):
        self.costmap = m

    def on_obs(self, m):
        self.obs = [(p.position.x, p.position.y, yaw_of(p.orientation)) for p in m.poses]

    # ---------- 幾何 ----------
    def wall_clearance(self, x, y):
        i = int((x - self.ox) / self.res)
        j = self.mh - 1 - int((y - self.oy) / self.res)
        if not (0 <= j < self.clr.shape[0] and 0 <= i < self.clr.shape[1]):
            return float('nan')
        return float(self.clr[j, i])

    def obstacle_clearance(self, x, y):
        """ロボット中心から、各動的障害物の箱の表面までの距離の最小値。"""
        best = float('inf')
        for (px, py, pyaw), d in zip(self.obs, self.dyn):
            c, s = math.cos(-pyaw), math.sin(-pyaw)
            dx, dy = x - px, y - py
            lx, ly = c * dx - s * dy, s * dx + c * dy
            hx, hy = d['size'][0] / 2, d['size'][1] / 2
            ex, ey = max(abs(lx) - hx, 0.0), max(abs(ly) - hy, 0.0)
            inside = min(hx - abs(lx), hy - abs(ly))
            best = min(best, math.hypot(ex, ey) if (ex or ey) else -inside)
        return best if best < float('inf') else float('nan')

    def dead_reckoned(self):
        if self.odom is None or self.odom0 is None:
            return None
        c0, s0 = math.cos(self.odom0[2]), math.sin(self.odom0[2])
        dx, dy = self.odom[0] - self.odom0[0], self.odom[1] - self.odom0[1]
        rx, ry = c0 * dx + s0 * dy, -s0 * dx + c0 * dy
        cg, sg = math.cos(self.gt0[2]), math.sin(self.gt0[2])
        return (self.gt0[0] + cg * rx - sg * ry, self.gt0[1] + sg * rx + cg * ry,
                wrap(self.gt0[2] + wrap(self.odom[2] - self.odom0[2])))

    def lookup(self, target, source):
        try:
            t = self.tf_buf.lookup_transform(target, source, rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y,
                    yaw_of(t.transform.rotation))
        except Exception:
            return None

    # ---------- 目標 ----------
    def send_goal(self):
        gx, gy = self.goals[self.gi]
        nx, ny = self.goals[(self.gi + 1) % len(self.goals)]
        yaw = math.atan2(ny - gy, nx - gx)
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = float(gx)
        g.pose.pose.position.y = float(gy)
        g.pose.pose.orientation.z = math.sin(yaw / 2)
        g.pose.pose.orientation.w = math.cos(yaw / 2)
        self.goal_active = True
        self.goal_t0 = self.now()
        self.events.append({'t': round(self.goal_t0, 2), 'ev': 'goal_sent',
                            'i': self.gi, 'xy': [gx, gy]})
        self.nav.send_goal_async(g).add_done_callback(self.on_accepted)

    def on_accepted(self, fut):
        h = fut.result()
        if not h.accepted:
            self.events.append({'t': round(self.now(), 2), 'ev': 'goal_rejected',
                                'i': self.gi})
            self.finish_goal(False)
            return
        self.goal_handle = h
        h.get_result_async().add_done_callback(self.on_result)

    def on_result(self, fut):
        status = fut.result().status
        ok = status == 4                      # STATUS_SUCCEEDED
        self.events.append({'t': round(self.now(), 2),
                            'ev': 'goal_done' if ok else 'goal_failed',
                            'i': self.gi, 'status': int(status),
                            'dt': round(self.now() - self.goal_t0, 2),
                            'err': round(math.dist(self.gt[:2], self.goals[self.gi]), 3)
                            if self.gt else None})
        self.finish_goal(ok)

    def finish_goal(self, ok):
        self.goal_active = False
        self.goal_handle = None
        self.gi += 1
        if self.gi >= len(self.goals):
            self.gi = 0
            self.laps += 1

    # ---------- kidnap ----------
    def kidnap(self):
        from gazebo_msgs.srv import SetEntityState
        req = SetEntityState.Request()
        req.state.name = 'ofl_bot'
        req.state.pose.position.x = float(self.a.kidnap_to[0])
        req.state.pose.position.y = float(self.a.kidnap_to[1])
        req.state.pose.position.z = 0.10
        yaw = float(self.a.kidnap_to[2])
        req.state.pose.orientation.z = math.sin(yaw / 2)
        req.state.pose.orientation.w = math.cos(yaw / 2)
        req.state.reference_frame = 'world'
        self.set_state.call_async(req)
        self.events.append({'t': round(self.now(), 2), 'ev': 'kidnap',
                            'to': list(self.a.kidnap_to)})
        self.get_logger().info(f'kidnapped to {self.a.kidnap_to}')

    # ---------- 本体 ----------
    def now(self):
        n = self.get_clock().now().nanoseconds * 1e-9
        return n - self.t0 if self.t0 is not None else 0.0

    def step(self):
        if self.gt is None or self.odom is None:
            return
        n = self.get_clock().now().nanoseconds * 1e-9
        import time as _time
        if self.t0 is None:
            self.t0 = n
            self.wall0 = _time.monotonic()
        t = n - self.t0
        wall = _time.monotonic() - self.wall0

        est = self.lookup('map', 'base_link')
        mo = self.lookup('map', 'odom')

        # 位置推定と Nav2 が揃うまでは目標を送らない (送っても TF 不在で失敗する)
        if self.nav_ready_t is None:
            if est is not None and self.nav.server_is_ready():
                self.nav_ready_t = t
                self.events.append({'t': round(t, 2), 'ev': 'nav_ready'})
                self.send_goal()
            elif t > 90.0:
                self.get_logger().error('nav2 / localization did not come up')
                self.done = True
                self.finish()
            return

        if (not self.kidnapped and self.a.kidnap_at is not None
                and t >= self.a.kidnap_at and self.set_state.service_is_ready()):
            self.kidnap()
            self.kidnapped = True

        if (not self.reinitialized and self.a.reinit_at is not None
                and t >= self.a.reinit_at and self.reinit.service_is_ready()):
            from std_srvs.srv import Empty
            self.reinit.call_async(Empty.Request())
            self.events.append({'t': round(t, 2), 'ev': 'reinitialize'})
            self.get_logger().info('called /reinitialize_global_localization')
            self.reinitialized = True

        # map -> odom の跳び (制御へ直接入る不連続)
        jm = jd = 0.0
        if mo is not None:
            if self.prev_mo is not None:
                jm = math.hypot(mo[0] - self.prev_mo[0], mo[1] - self.prev_mo[1])
                jd = abs(wrap(mo[2] - self.prev_mo[2])) * 180 / math.pi
            self.prev_mo = mo

        # 制御へ入る不連続は map->base の 1 ステップ変化そのものではなく、
        # そこから実際の運動を除いた分 = 推定誤差ベクトルの変化である。
        # (map->odom の変化はオドメトリ側のスリップも一緒に拾ってしまう)
        pe = ye = float('nan')
        ej = ejd = 0.0
        if est is not None:
            ex, ey = est[0] - self.gt[0], est[1] - self.gt[1]
            eth = wrap(est[2] - self.gt[2])
            pe = math.hypot(ex, ey)
            ye = abs(eth) * 180 / math.pi
            if self.prev_err is not None:
                ej = math.hypot(ex - self.prev_err[0], ey - self.prev_err[1])
                ejd = abs(wrap(eth - self.prev_err[2])) * 180 / math.pi
            self.prev_err = (ex, ey, eth)
        dr = self.dead_reckoned()
        oe = math.hypot(dr[0] - self.gt[0], dr[1] - self.gt[1]) if dr else float('nan')

        def f(v, n=4):
            return round(v, n) if v == v else 'nan'
        self.rows.append([
            round(t, 3), *[round(v, 4) for v in self.gt],
            *([round(v, 4) for v in est] if est else ['nan'] * 3),
            f(pe), f(ye, 2),
            *([round(v, 4) for v in dr] if dr else ['nan'] * 3), f(oe),
            f(self.wall_clearance(*self.gt[:2]), 3),
            f(self.obstacle_clearance(*self.gt[:2]), 3),
            round(self.cmd[0], 3), round(self.cmd[1], 3),
            self.gi, round(jm, 4), round(jd, 2), round(ej, 4), round(ejd, 2),
            1 if self.kidnapped else 0, round(wall, 2)])

        if not self.goal_active and not self.done:
            self.send_goal()

        if t >= self.a.duration and not self.done:
            self.done = True
            self.finish()

    def finish(self):
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
        self.stop_pub.publish(Twist())
        with open(self.a.out, 'w', newline='') as fh:
            wr = csv.writer(fh)
            wr.writerow(['t', 'gt_x', 'gt_y', 'gt_yaw', 'est_x', 'est_y', 'est_yaw',
                         'pos_err', 'yaw_err', 'dr_x', 'dr_y', 'dr_yaw', 'odom_err',
                         'wall_clr', 'obs_clr', 'cmd_v', 'cmd_w', 'goal_idx',
                         'tf_jump_m', 'tf_jump_deg', 'err_jump_m', 'err_jump_deg',
                         'kidnapped', 'wall'])
            wr.writerows(self.rows)
        if self.costmap is not None:
            g = self.costmap
            img = np.array(g.data, dtype=np.int16).reshape(g.info.height, g.info.width)
            img = np.where(img < 0, 128, (255 - img * 255 // 100)).astype(np.uint8)
            img = np.flipud(img)      # 行 0 = 最大 y (map_server 規約に合わせる)
            base = os.path.splitext(self.a.out)[0] + '_costmap'
            with open(base + '.pgm', 'wb') as fh:
                fh.write(b'P5\n%d %d\n255\n' % (g.info.width, g.info.height))
                fh.write(img.tobytes())
            with open(base + '.yaml', 'w') as fh:
                fh.write(f'image: {os.path.basename(base)}.pgm\n'
                         f'resolution: {g.info.resolution}\n'
                         f'origin: [{g.info.origin.position.x}, '
                         f'{g.info.origin.position.y}, 0.0]\n'
                         'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        with open(os.path.splitext(self.a.out)[0] + '_events.json', 'w') as fh:
            json.dump({'events': self.events, 'goals': self.goals,
                       'laps': self.laps, 'nav_ready_t': self.nav_ready_t}, fh, indent=2)
        self.get_logger().info(f'wrote {self.a.out} ({len(self.rows)} rows, '
                               f'{self.laps} laps, {len(self.events)} events)')
        # rclpy.shutdown() をコールバックから呼んでも spin が抜けず、プロセスが
        # ハード timeout まで残る (ActionClient / TransformListener が居ると
        # 実際にそうなった)。出力は既に閉じてあるので、ここで確実に落とす。
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)


def pick_goals(route, spacing):
    """計画済み経路を弧長 spacing で等間隔にリサンプルして目標にする。

    経路そのものの上に載るので (経路はクリアランス 0.9 m 以上で計画済み)、
    目標が壁の中や到達不能な場所に落ちることがない。
    """
    seglen = [math.dist(a, b) for a, b in zip(route[:-1], route[1:])]
    total = sum(seglen)
    n = max(3, int(round(total / spacing)))
    step = total / n
    goals, target, acc, k = [route[0]], step, 0.0, 0
    for (a, b), L in zip(zip(route[:-1], route[1:]), seglen):
        while target <= acc + L + 1e-9 and len(goals) < n:
            u = (target - acc) / L if L > 1e-9 else 0.0
            goals.append((round(a[0] + u * (b[0] - a[0]), 3),
                          round(a[1] + u * (b[1] - a[1]), 3)))
            target += step
        acc += L
    return goals


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--route', required=True)
    ap.add_argument('--map', required=True)
    ap.add_argument('--dynamic', default='')
    ap.add_argument('--out', required=True)
    ap.add_argument('--duration', type=float, default=300.0)
    ap.add_argument('--goal-spacing', type=float, default=11.0)
    ap.add_argument('--dump-costmap', action='store_true',
                    help='最後の大域 costmap を PGM で保存する')
    ap.add_argument('--kidnap-at', type=float, default=None)
    ap.add_argument('--reinit-at', type=float, default=None,
                    help='この時刻に /reinitialize_global_localization を呼ぶ '
                         '(オラクル検知を仮定した一様撒き直し)')
    ap.add_argument('--kidnap-to', type=float, nargs=3, default=[20.0, 3.5, 1.57])
    a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    route = [tuple(p) for p in json.load(open(a.route))['route']]
    goals = pick_goals(route, a.goal_spacing)
    rclpy.init(args=sys.argv)
    node = Nav2Driver(a, goals)
    node.get_logger().info(f'{len(goals)} goals: {goals}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
