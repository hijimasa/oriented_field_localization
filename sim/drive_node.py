#!/usr/bin/env python3
"""経路を追従する自動運転と、位置推定の誤差記録。

**制御には真値 (/ground_truth) を使う。**制御器の出来は評価対象ではないので、
位置推定の誤差が走行の破綻に跳ね返らないようにする (位置推定の出力で走ると、
一度外した瞬間に壁へ突っ込んで以降の評価が意味を失う)。位置推定は完全に
受け身の観測者として評価する。

記録するもの (CSV):
  t, gt_x, gt_y, gt_yaw, est_x, est_y, est_yaw, pos_err, yaw_err,
  odom_x, odom_y, odom_yaw, odom_err, age
  (odom_err = オドメトリだけを積分した場合の誤差。位置推定が「何を直しているか」の基準)

kidnap: --kidnap-at <秒> を指定すると、その時刻に真値をずらしてロボットを瞬間移動
させる (/gazebo/set_entity_state)。位置推定が GLOBAL へ落ちて復帰できるかを見る。
"""
import argparse
import csv
import json
import math
import os
import sys

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Driver(Node):
    def __init__(self, route, out_csv, duration, kidnap_at, kidnap_to):
        super().__init__('ofl_sim_driver')   # use_sim_time は --ros-args で渡す
        self.route = route
        self.wp = 0
        self.laps = 0
        self.duration = duration
        self.kidnap_at = kidnap_at
        self.kidnap_to = kidnap_to
        self.kidnapped = False
        self.done = False
        self.t0 = None

        self.gt = None
        self.gt0 = None      # 最初の真値 (デッドレコニングの原点)
        self.odom = None
        self.odom0 = None    # 最初のオドメトリ
        self.est = None
        self.est_t = None
        self.rows = []
        self.out_csv = out_csv

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, 'ground_truth', self.on_gt, qos)
        self.create_subscription(Odometry, 'odom', self.on_odom, qos)
        self.create_subscription(
            PoseWithCovarianceStamped, '/oriented_field_localization/pose', self.on_est, 10)
        self.cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.set_state = self.create_client(
            __import__('gazebo_msgs.srv', fromlist=['SetEntityState']).SetEntityState,
            '/gazebo/set_entity_state')
        self.create_timer(0.05, self.step)

    def on_gt(self, m):
        self.gt = (m.pose.pose.position.x, m.pose.pose.position.y,
                   yaw_of(m.pose.pose.orientation))

    def on_odom(self, m):
        self.odom = (m.pose.pose.position.x, m.pose.pose.position.y,
                     yaw_of(m.pose.pose.orientation))
        if self.odom0 is None and self.gt is not None:
            self.odom0 = self.odom
            self.gt0 = self.gt

    def dead_reckoned(self):
        """オドメトリだけを積分した場合のワールド姿勢。

        odom は原点始まりの独自座標なので、そのまま真値と比べても意味が無い。
        最初の真値を原点に置いて odom の相対変位を掛ける
        (gt0 * odom0^-1 * odom_t) のが「オドメトリだけで走った場合」に当たる。
        """
        if self.odom is None or self.odom0 is None:
            return None
        c0, s0 = math.cos(self.odom0[2]), math.sin(self.odom0[2])
        dx, dy = self.odom[0] - self.odom0[0], self.odom[1] - self.odom0[1]
        rx, ry = c0 * dx + s0 * dy, -s0 * dx + c0 * dy
        dth = wrap(self.odom[2] - self.odom0[2])
        cg, sg = math.cos(self.gt0[2]), math.sin(self.gt0[2])
        return (self.gt0[0] + cg * rx - sg * ry,
                self.gt0[1] + sg * rx + cg * ry,
                wrap(self.gt0[2] + dth))

    def on_est(self, m):
        self.est = (m.pose.pose.position.x, m.pose.pose.position.y,
                    yaw_of(m.pose.pose.orientation))
        self.est_t = self.get_clock().now().nanoseconds * 1e-9

    def kidnap(self):
        from gazebo_msgs.srv import SetEntityState
        req = SetEntityState.Request()
        req.state.name = 'ofl_bot'
        req.state.pose.position.x = float(self.kidnap_to[0])
        req.state.pose.position.y = float(self.kidnap_to[1])
        req.state.pose.position.z = 0.05
        yaw = float(self.kidnap_to[2])
        req.state.pose.orientation.z = math.sin(yaw / 2)
        req.state.pose.orientation.w = math.cos(yaw / 2)
        req.state.reference_frame = 'world'
        self.set_state.call_async(req)
        self.get_logger().info(f'kidnapped to {self.kidnap_to}')

    def step(self):
        if self.gt is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0

        if (not self.kidnapped and self.kidnap_at is not None and t >= self.kidnap_at
                and self.set_state.service_is_ready()):
            self.kidnap()
            self.kidnapped = True
            # 瞬間移動の直後は制御も追従できないので、最近傍の waypoint へ寄せる
            gx, gy = self.kidnap_to[0], self.kidnap_to[1]
            self.wp = min(range(len(self.route)),
                          key=lambda i: (self.route[i][0] - gx) ** 2 + (self.route[i][1] - gy) ** 2)

        # ---- 記録 ----
        if self.est is not None:
            pe = math.hypot(self.est[0] - self.gt[0], self.est[1] - self.gt[1])
            ye = abs(wrap(self.est[2] - self.gt[2])) * 180 / math.pi
        else:
            pe, ye = float('nan'), float('nan')
        dr = self.dead_reckoned()
        oe = math.hypot(dr[0] - self.gt[0], dr[1] - self.gt[1]) if dr else float('nan')
        self.rows.append([
            round(t, 3), *[round(v, 4) for v in self.gt],
            *([round(v, 4) for v in self.est] if self.est else ['nan'] * 3),
            round(pe, 4) if pe == pe else 'nan', round(ye, 2) if ye == ye else 'nan',
            *([round(v, 4) for v in dr] if dr else ['nan'] * 3),
            round(oe, 4) if oe == oe else 'nan',
            round(now - self.est_t, 3) if self.est_t else 'nan',
            1 if self.kidnapped else 0])

        # ---- 経路追従 (真値ベース) ----
        gx, gy = self.route[self.wp]
        dx, dy = gx - self.gt[0], gy - self.gt[1]
        d = math.hypot(dx, dy)
        if d < 0.35:
            self.wp += 1
            if self.wp >= len(self.route):
                self.wp = 0
                self.laps += 1
            gx, gy = self.route[self.wp]
            dx, dy = gx - self.gt[0], gy - self.gt[1]
            d = math.hypot(dx, dy)
        heading = wrap(math.atan2(dy, dx) - self.gt[2])
        cmd = Twist()
        cmd.angular.z = max(-1.2, min(1.2, 1.8 * heading))
        cmd.linear.x = 0.0 if abs(heading) > 0.8 else min(0.6, 0.9 * d)
        self.cmd.publish(cmd)

        if t >= self.duration and not self.done:
            # finish() を 2 回呼ぶと rclpy.shutdown() が例外を投げ、spin が
            # 抜けずにプロセスが残る (タイマは以後も回り続けるため)
            self.done = True
            self.finish()

    def finish(self):
        self.cmd.publish(Twist())
        with open(self.out_csv, 'w', newline='') as f:
            wr = csv.writer(f)
            wr.writerow(['t', 'gt_x', 'gt_y', 'gt_yaw', 'est_x', 'est_y', 'est_yaw',
                         'pos_err', 'yaw_err', 'dr_x', 'dr_y', 'dr_yaw',
                         'odom_err', 'est_age', 'kidnapped'])
            wr.writerows(self.rows)
        self.get_logger().info(f'wrote {self.out_csv} ({len(self.rows)} rows, '
                               f'{self.laps} laps)')
        # コールバックから rclpy.shutdown() を呼んでも spin が抜けないことがあり、
        # その場合プロセスはハード timeout まで残る。出力は閉じてあるので落とす。
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--route', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--duration', type=float, default=180.0)
    ap.add_argument('--kidnap-at', type=float, default=None)
    ap.add_argument('--kidnap-to', type=float, nargs=3, default=[20.0, 3.5, 1.57])
    # --ros-args 以降は rclpy が使うので argparse から外す
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    a = ap.parse_args(argv)
    route = json.load(open(a.route))['route']
    rclpy.init(args=sys.argv)
    node = Driver(route, a.out, a.duration, a.kidnap_at, a.kidnap_to)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.finish()


if __name__ == '__main__':
    main()
