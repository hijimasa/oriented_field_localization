#!/usr/bin/env python3
"""Gazebo の world と、それと厳密に一致する占有格子地図を 1 か所から生成する。

壁の定義 (線分の集合) を唯一の出所にして

  - Gazebo Classic の world SDF (壁を box link として置く)
  - map_server 形式の占有格子 (PGM + YAML)
  - 走行経路の waypoint (JSON)

を同時に出す。world と地図がずれると位置推定の評価が意味を失うので、
両者を別々に用意しない。

usage: make_env.py <out_dir>
"""
import argparse
import json
import math
import os

import numpy as np

WALL_H = 1.0      # 壁の高さ [m]
WALL_T = 0.15     # 壁の厚さ [m]
RES = 0.05        # 地図の解像度 [m/px]
PAD = 1.0         # 地図の外周余白 [m]


def build_walls():
    """壁の線分 [(x1, y1, x2, y2), ...] と走行経路を返す。

    24 x 18 m のオフィス。外周、3 つの部屋、斜めの間仕切り、柱を持つ。
    自己相似を避けるため部屋の大きさと開口位置は非対称にしてある
    (対称な環境では 180 度回した姿勢が同じ観測を与えてしまう)。
    """
    W, H = 24.0, 18.0
    w = []

    def rect(x0, y0, x1, y1):
        w.extend([(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)])

    # 外周
    rect(0, 0, W, H)
    # 左上の部屋 (南側に開口)
    w += [(1.5, 11.0, 7.0, 11.0), (7.0, 11.0, 7.0, 16.5),
          (1.5, 11.0, 1.5, 16.5)]
    # 右上の部屋 (西側に 2.5 m の開口)
    w += [(15.0, 11.5, 22.5, 11.5), (15.0, 11.5, 15.0, 13.0),
          (15.0, 15.5, 15.0, 16.5)]
    # 右下の部屋 (北側に開口)
    w += [(16.5, 1.5, 16.5, 7.0), (16.5, 7.0, 19.0, 7.0),
          (21.0, 7.0, 22.5, 7.0)]
    # 中央の斜め間仕切り (直交壁だけだと投影の主方位が 2 本しか立たない)
    w += [(8.5, 3.0, 13.0, 7.5)]
    # 左下の什器
    w += [(2.0, 3.0, 5.5, 3.0), (5.5, 3.0, 5.5, 5.0)]
    # 柱 (小さな正方形)
    for cx, cy in [(11.0, 13.5), (19.5, 9.0), (6.5, 8.0)]:
        rect(cx - 0.25, cy - 0.25, cx + 0.25, cy + 0.25)

    # 経路の通過点 (アンカー)。実際の経路は地図の自由空間上で計画するので、
    # ここは「どこを通したいか」だけを指定する。
    anchors = [(3.0, 8.0), (12.0, 9.3), (20.0, 9.3), (17.0, 13.5),
               (9.0, 13.5), (11.0, 9.0), (13.5, 2.5), (3.0, 2.0), (3.0, 8.0)]
    start = (3.0, 8.0, 0.0)
    return w, anchors, start, W, H


def build_dynamic():
    """動的障害物の定義を返す。**地図には焼かない** (未知の障害物である)。

    経路の走行帯を横切る/沿って動くように置いてある。静止させて置くと
    「地図に無い壁」でしかないので、往復させて遮蔽が入れ替わるようにする。
    kinematic なリンクにしてあるので、物理では動かされず経路は完全に再現する
    (位置推定の条件を変えても障害物の軌跡が同一でないと比較にならない)。
    """
    return [
        # 東西の走行帯を横切る人
        {'name': 'walker0', 'size': [0.40, 0.40, 1.20], 'speed': 0.9,
         'path': [[9.0, 7.4], [9.0, 10.8]]},
        # 走行帯に沿って往復する人 (正面から来て、追い越される)
        {'name': 'walker1', 'size': [0.40, 0.40, 1.20], 'speed': 0.8,
         'path': [[14.0, 9.7], [18.4, 9.7]]},
        # 上の通路を横切る人
        {'name': 'walker2', 'size': [0.40, 0.40, 1.20], 'speed': 0.7,
         'path': [[13.0, 12.4], [13.0, 14.6]]},
        # 南の走行帯を塞ぐように往復する台車 (大きく、遅い)
        {'name': 'cart0', 'size': [0.70, 0.50, 0.90], 'speed': 0.45,
         'path': [[6.5, 1.9], [11.0, 1.9]]},
    ]


def dynamic_sdf(dyn):
    """動的障害物を kinematic なリンクとして world へ足す。

    static にすると set_entity_state で動かしても衝突形状が追随しない実装が
    あるため、非 static + kinematic (物理では動かないが衝突はする) にする。
    """
    parts = []
    for d in dyn:
        sx, sy, sz = d['size']
        x, y = d['path'][0]
        parts.append(f'''    <model name="{d['name']}">
      <pose>{x:.3f} {y:.3f} {sz/2:.3f} 0 0 0</pose>
      <link name="body">
        <kinematic>true</kinematic>
        <inertial><mass>30.0</mass><inertia><ixx>3.0</ixx><iyy>3.0</iyy>
          <izz>3.0</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="c"><geometry><box><size>{sx} {sy} {sz}</size></box>
          </geometry></collision>
        <visual name="v"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><ambient>0.8 0.3 0.2 1</ambient>
          <diffuse>0.9 0.4 0.25 1</diffuse></material></visual>
      </link>
    </model>''')
    return parts


def world_sdf(walls, W, H, dyn=()):
    parts = ['<?xml version="1.0" ?>',
             '<sdf version="1.6">', '  <world name="office">',
             '    <include><uri>model://sun</uri></include>',
             '    <include><uri>model://ground_plane</uri></include>',
             # real_time_update_rate=0 は「できるだけ速く」で、実時間の数百倍で
             # 走ってしまい位置推定も制御も追従できない。step 0.002 s に対して
             # 500 iter/s = 実時間の 1.0 倍に固定する。
             '    <physics type="ode"><max_step_size>0.002</max_step_size>'
             '<real_time_update_rate>500</real_time_update_rate></physics>',
             # gazebo_ros_init / gazebo_ros_factory は **system plugin** なので
             # world SDF ではなく gzserver -s で渡す (run_sim.sh を参照)。
             # gazebo_ros_state は world plugin なのでここに書く。
             '    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">'
             '<ros><namespace>/gazebo</namespace></ros><update_rate>10.0</update_rate></plugin>',
             '    <model name="walls"><static>true</static>']
    for i, (x1, y1, x2, y2) in enumerate(walls):
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        length = math.hypot(x2 - x1, y2 - y1)
        yaw = math.atan2(y2 - y1, x2 - x1)
        if length < 1e-6:
            continue
        parts.append(f'''      <link name="w{i}">
        <pose>{cx:.4f} {cy:.4f} {WALL_H/2:.3f} 0 0 {yaw:.6f}</pose>
        <collision name="c"><geometry><box><size>{length:.4f} {WALL_T} {WALL_H}</size>
          </box></geometry></collision>
        <visual name="v"><geometry><box><size>{length:.4f} {WALL_T} {WALL_H}</size>
          </box></geometry><material><ambient>0.6 0.6 0.62 1</ambient>
          <diffuse>0.7 0.7 0.72 1</diffuse></material></visual>
      </link>''')
    parts += ['    </model>']
    parts += dynamic_sdf(dyn)
    parts += ['  </world>', '</sdf>']
    return '\n'.join(parts)


def occupancy_map(walls, W, H):
    """world と同じ線分を、同じ厚さで占有格子へ焼く。"""
    ox, oy = -PAD, -PAD
    nx = int(math.ceil((W + 2 * PAD) / RES))
    ny = int(math.ceil((H + 2 * PAD) / RES))
    img = np.full((ny, nx), 254, dtype=np.uint8)   # free
    half = WALL_T / 2.0
    for (x1, y1, x2, y2) in walls:
        dx, dy = x2 - x1, y2 - y1
        n = max(2, int(math.hypot(dx, dy) / (RES * 0.4)))
        for k in range(n + 1):
            t = k / n
            px, py = x1 + t * dx, y1 + t * dy
            # 厚さぶんの正方形で塗る (world の box と同じ幅になる)
            i0 = int((px - half - ox) / RES); i1 = int((px + half - ox) / RES)
            j0 = int((py - half - oy) / RES); j1 = int((py + half - oy) / RES)
            for j in range(max(0, j0), min(ny - 1, j1) + 1):
                for i in range(max(0, i0), min(nx - 1, i1) + 1):
                    img[ny - 1 - j, i] = 0        # 行 0 = 最大 y (map_server 規約)
    return img, ox, oy, RES


def clearance_map(img, res):
    """占有画素からの距離 [m] を BFS で作る (4 近傍、格子単位)。"""
    from collections import deque
    h, w = img.shape
    INF = 1 << 30
    dist = np.full((h, w), INF, dtype=np.int32)
    q = deque()
    ys, xs = np.nonzero(img == 0)
    for y, x in zip(ys.tolist(), xs.tolist()):
        dist[y, x] = 0
        q.append((y, x))
    while q:
        y, x = q.popleft()
        d = dist[y, x] + 1
        for ny, nx in ((y + 1, x), (y - 1, x), (y, x + 1), (y, x - 1)):
            if 0 <= ny < h and 0 <= nx < w and dist[ny, nx] > d:
                dist[ny, nx] = d
                q.append((ny, nx))
    return dist.astype(np.float32) * res


def plan_route(img, clr, ox, oy, res, anchors, min_clear):
    """クリアランス min_clear 以上のセルだけを通って anchors を順に結ぶ。

    world と地図が同じ壁から作られているので、この経路は Gazebo でも通れる。
    """
    from collections import deque
    h, w = img.shape

    def to_cell(p):
        return (h - 1 - int((p[1] - oy) / res), int((p[0] - ox) / res))

    def to_world(c):
        return ((c[1] + 0.5) * res + ox, (h - 1 - c[0] + 0.5) * res + oy)

    free = clr >= min_clear

    def nearest_free(c):
        if free[c]:
            return c
        best, bd = None, 1e18
        ys, xs = np.nonzero(free)
        d = (ys - c[0]) ** 2 + (xs - c[1]) ** 2
        k = int(np.argmin(d))
        best, bd = (int(ys[k]), int(xs[k])), float(d[k])
        return best

    path_cells = []
    for a, b in zip(anchors[:-1], anchors[1:]):
        s_, g_ = nearest_free(to_cell(a)), nearest_free(to_cell(b))
        prev = {s_: None}
        q = deque([s_])
        while q:
            cur = q.popleft()
            if cur == g_:
                break
            y, x = cur
            for nxt in ((y + 1, x), (y - 1, x), (y, x + 1), (y, x - 1)):
                if 0 <= nxt[0] < h and 0 <= nxt[1] < w and free[nxt] and nxt not in prev:
                    prev[nxt] = cur
                    q.append(nxt)
        if g_ not in prev:
            raise SystemExit(f"経路が見つからない: {a} -> {b} (min_clear={min_clear})")
        seg = []
        cur = g_
        while cur is not None:
            seg.append(cur)
            cur = prev[cur]
        seg.reverse()
        path_cells += seg[1:] if path_cells else seg
    pts = [to_world(c) for c in path_cells]

    # Douglas-Peucker で間引く (走行の形は変えず waypoint 数だけ減らす)
    def dp(ps, eps):
        if len(ps) < 3:
            return ps
        x0, y0 = ps[0]
        x1, y1 = ps[-1]
        dx, dy = x1 - x0, y1 - y0
        n = math.hypot(dx, dy)
        best_i, best_d = 0, -1.0
        for i in range(1, len(ps) - 1):
            px, py = ps[i]
            d = (abs(dy * px - dx * py + x1 * y0 - y1 * x0) / n) if n > 1e-9 \
                else math.hypot(px - x0, py - y0)
            if d > best_d:
                best_i, best_d = i, d
        if best_d <= eps:
            return [ps[0], ps[-1]]
        return dp(ps[:best_i + 1], eps)[:-1] + dp(ps[best_i:], eps)

    return [(round(x, 3), round(y, 3)) for x, y in dp(pts, 0.15)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out_dir')
    ap.add_argument('--min-clearance', type=float, default=0.9,
                    help='経路が壁から確保するクリアランス [m]')
    ap.add_argument('--dynamic', action='store_true',
                    help='動的障害物を world に置く (地図には焼かない)')
    a = ap.parse_args()
    os.makedirs(a.out_dir, exist_ok=True)

    walls, anchors, start, W, H = build_walls()

    dyn = build_dynamic() if a.dynamic else []
    with open(os.path.join(a.out_dir, 'office.world'), 'w') as f:
        f.write(world_sdf(walls, W, H, dyn))
    with open(os.path.join(a.out_dir, 'dynamic.json'), 'w') as f:
        json.dump(dyn, f, indent=2)

    img, ox, oy, res = occupancy_map(walls, W, H)
    clr = clearance_map(img, res)
    route = plan_route(img, clr, ox, oy, res, anchors, a.min_clearance)
    worst = min(
        clr[img.shape[0] - 1 - int((y - oy) / res), int((x - ox) / res)] for x, y in route)
    pgm = os.path.join(a.out_dir, 'office.pgm')
    with open(pgm, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (img.shape[1], img.shape[0]))
        f.write(img.tobytes())
    with open(os.path.join(a.out_dir, 'office.yaml'), 'w') as f:
        f.write(f"image: office.pgm\nresolution: {res}\n"
                f"origin: [{ox}, {oy}, 0.0]\nnegate: 0\n"
                "occupied_thresh: 0.65\nfree_thresh: 0.196\n")
    with open(os.path.join(a.out_dir, 'route.json'), 'w') as f:
        json.dump({'route': route, 'start': start, 'size': [W, H]}, f, indent=2)

    occ = int((img == 0).sum())
    print(f"world: {len(walls)} wall segments")
    print(f"map:   {img.shape[1]}x{img.shape[0]} @ {res} m/px, "
          f"origin ({ox}, {oy}), occupied {occ} px ({100*occ/img.size:.2f}%)")
    length = sum(math.hypot(route[k + 1][0] - route[k][0], route[k + 1][1] - route[k][1])
                 for k in range(len(route) - 1))
    print(f"route: {len(route)} waypoints, {length:.1f} m, "
          f"min clearance {worst:.2f} m (>= {a.min_clearance})")
    if dyn:
        print(f"dynamic: {len(dyn)} obstacles (地図には含めない)")


if __name__ == '__main__':
    main()
