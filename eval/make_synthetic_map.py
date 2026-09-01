#!/usr/bin/env python3
"""検証用の合成占有格子地図を生成する (numpy のみ、第三者データセット不要)。

このハーネスで公開されている数値は Intel Research Lab 地図 (Radish) で測ったものだが、
その地図は再配布条件を確認していないので本リポジトリには同梱しない (README 参照)。
かわりに、誰でも同じものを再生成できる合成地図を用意する。目的は次の 2 つである。

  1. ハーネスが動くこと自体の検証 (ビルド・実行・集計が最後まで通るか)。
  2. Radon と BBS を同一スキャンで比較できること。

生成される地図は Intel と同じ寸法感 (30 x 30 m, 0.02 m/px) の室内で、
廊下と大きさの異なる部屋、開口部、柱を持つ。自己相似を避けるため部屋の配置と
開口位置は決定的な擬似乱数で崩してある (seed 固定なので毎回同一)。

usage: make_synthetic_map.py [out_dir] [--seed N]
出力: <out_dir>/synthetic.pgm, <out_dir>/synthetic.yaml
"""
import argparse
import os

import numpy as np

FREE, OCC, UNKNOWN = 254, 0, 205


def build(res=0.02, size_m=30.0, seed=7):
    rng = np.random.default_rng(seed)
    n = int(round(size_m / res))
    img = np.full((n, n), UNKNOWN, dtype=np.uint8)

    def m2p(v):
        return int(round(v / res))

    wall = max(1, m2p(0.15))          # 壁厚 0.15 m

    def fill(x0, y0, x1, y1, val):
        img[max(0, m2p(y0)):m2p(y1), max(0, m2p(x0)):m2p(x1)] = val

    def room(x0, y0, x1, y1, doors):
        """外周を壁、内部を自由空間にし、doors=[(辺, 中心[m], 幅[m])] を開口する。"""
        fill(x0, y0, x1, y1, OCC)
        fill(x0 + 0.15, y0 + 0.15, x1 - 0.15, y1 - 0.15, FREE)
        for side, c, w in doors:
            if side in ("N", "S"):
                y = y1 - 0.15 if side == "N" else y0
                fill(c - w / 2, y, c + w / 2, y + 0.15, FREE)
            else:
                x = x1 - 0.15 if side == "E" else x0
                fill(x, c - w / 2, x + 0.15, c + w / 2, FREE)

    # 外周 + 内部の自由空間
    fill(1.0, 1.0, size_m - 1.0, size_m - 1.0, OCC)
    fill(1.0 + 0.15, 1.0 + 0.15, size_m - 1.15, size_m - 1.15, FREE)

    # 十字の廊下を残し、その周りに部屋を置く
    jitter = lambda s: float(rng.uniform(-s, s))
    rooms = [
        (1.15, 1.15, 9.0 + jitter(0.6), 11.0 + jitter(0.8), [("E", 6.0, 1.1)]),
        (1.15, 18.0 + jitter(0.7), 10.5 + jitter(0.5), size_m - 1.15, [("S", 5.0, 1.0)]),
        (20.0 + jitter(0.6), 1.15, size_m - 1.15, 9.5 + jitter(0.7), [("W", 5.0, 1.2)]),
        (19.0 + jitter(0.5), 17.0 + jitter(0.6), size_m - 1.15, size_m - 1.15,
         [("W", 22.0, 1.0), ("S", 24.0, 1.1)]),
        (12.0 + jitter(0.4), 3.0 + jitter(0.4), 17.5, 8.0, [("N", 15.0, 0.9)]),
    ]
    for x0, y0, x1, y1, doors in rooms:
        room(x0, y0, x1, y1, doors)

    # 柱 (方位ごとの投影構造に非対称性を足す)
    for cx, cy in [(14.0, 14.5), (17.5, 21.0), (9.5, 15.0), (23.0, 13.0)]:
        cx += jitter(0.3)
        cy += jitter(0.3)
        fill(cx - 0.2, cy - 0.2, cx + 0.2, cy + 0.2, OCC)

    # 壁の一部を欠いた開口 (廊下の見通しを場所で変える)
    fill(12.0, 12.0, 12.0 + 0.15, 16.5, OCC)
    fill(12.0, 14.0, 12.0 + 0.15, 15.0, FREE)

    def seg(x0, y0, x1, y1, t=0.15):
        """任意角度の壁。軸平行だけの地図はサイノグラムの主方位が 2 本しか
        立たず、離れた姿勢が同じ投影構造を持ちやすい。斜め壁で方位を散らす。"""
        steps = int(max(abs(x1 - x0), abs(y1 - y0)) / res) + 1
        for k in range(steps + 1):
            u = k / steps
            fill(x0 + (x1 - x0) * u - t / 2, y0 + (y1 - y0) * u - t / 2,
                 x0 + (x1 - x0) * u + t / 2, y0 + (y1 - y0) * u + t / 2, OCC)

    # 斜めの間仕切り
    seg(10.8, 12.2, 14.5, 9.2)
    seg(18.5, 12.0, 22.5, 15.4)
    seg(3.0, 13.5, 6.4, 16.6)

    # 什器相当のクラッタ。実地図の識別性は家具・什器の細かい構造に多く由来する。
    # これが無いと矩形の部屋が互いに区別できず、自己相似で誤マッチが増える。
    free_mask = img == FREE
    placed = 0
    guard = 0
    while placed < 90 and guard < 4000:
        guard += 1
        cx = float(rng.uniform(1.5, size_m - 1.5))
        cy = float(rng.uniform(1.5, size_m - 1.5))
        w = float(rng.uniform(0.25, 0.9))
        h = float(rng.uniform(0.25, 0.9))
        px0, py0 = m2p(cx - w / 2), m2p(cy - h / 2)
        px1, py1 = m2p(cx + w / 2), m2p(cy + h / 2)
        if px0 < 0 or py0 < 0 or px1 >= n or py1 >= n:
            continue
        pad = m2p(0.35)
        if not free_mask[max(0, py0 - pad):py1 + pad, max(0, px0 - pad):px1 + pad].all():
            continue
        img[py0:py1, px0:px1] = OCC
        free_mask[py0:py1, px0:px1] = False
        placed += 1

    _ = wall
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out_dir", nargs="?", default="maps")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--res", type=float, default=0.02)
    ap.add_argument("--size", type=float, default=30.0)
    a = ap.parse_args()

    img = build(res=a.res, size_m=a.size, seed=a.seed)
    os.makedirs(a.out_dir, exist_ok=True)
    pgm = os.path.join(a.out_dir, "synthetic.pgm")
    # PGM は左上原点、占有格子は左下原点なので上下反転して書く
    with open(pgm, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (img.shape[1], img.shape[0]))
        f.write(np.flipud(img).tobytes())
    yml = os.path.join(a.out_dir, "synthetic.yaml")
    with open(yml, "w") as f:
        f.write(
            "image: synthetic.pgm\n"
            f"resolution: {a.res}\n"
            "origin: [-10.0, -23.0, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n"
        )
    occ = int((img == OCC).sum())
    free = int((img == FREE).sum())
    print(f"wrote {pgm} ({img.shape[1]}x{img.shape[0]}, {a.res} m/px)")
    print(f"      occupied {occ} px ({100*occ/img.size:.2f}%), free {free} px")
    print(f"wrote {yml}")


if __name__ == "__main__":
    main()
