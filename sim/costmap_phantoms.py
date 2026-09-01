#!/usr/bin/env python3
"""大域 costmap に焼き付いた「地図に無い障害物」を数える。

位置推定が外れている間に取り込んだスキャンは、**誤った場所へ壁として
記録される**。それは実際には何も無い場所なので、以後そこを通るレイで
クリアされる機会が無く、costmap に残り続けて経路計画を歪める。
位置推定の失敗が航法に残す傷はここにしか現れないので、静的地図と重ねて数える。

usage: costmap_phantoms.py <costmap.yaml> <static_map.yaml> [--dilate-m 0.6]
"""
import argparse
import math
import os

import numpy as np


def load(yaml_path):
    res, ox, oy, image = 0.05, 0.0, 0.0, None
    for line in open(yaml_path):
        if line.startswith('image:'):
            image = line.split(':', 1)[1].strip()
        elif line.startswith('resolution:'):
            res = float(line.split(':', 1)[1])
        elif line.startswith('origin:'):
            v = line.split('[', 1)[1].split(']')[0].split(',')
            ox, oy = float(v[0]), float(v[1])
    with open(os.path.join(os.path.dirname(yaml_path), image), 'rb') as f:
        assert f.readline().strip() == b'P5'
        w, h = map(int, f.readline().split())
        f.readline()
        img = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w).copy()
    return img, res, ox, oy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('costmap')
    ap.add_argument('static_map')
    ap.add_argument('--dilate-m', type=float, default=0.6,
                    help='静的な壁の近傍とみなす距離 [m] (膨張ぶんを除くため)')
    a = ap.parse_args()
    cm, cres, cox, coy = load(a.costmap)
    sm, sres, sox, soy = load(a.static_map)

    # 静的な壁から dilate_m 以内かどうかのマスク (BFS の距離場)
    from collections import deque
    occ = sm == 0
    dist = np.full(sm.shape, 1 << 30, dtype=np.int32)
    q = deque()
    for y, x in zip(*np.nonzero(occ)):
        dist[y, x] = 0
        q.append((y, x))
    while q:
        y, x = q.popleft()
        d = dist[y, x] + 1
        for ny, nx in ((y+1, x), (y-1, x), (y, x+1), (y, x-1)):
            if 0 <= ny < sm.shape[0] and 0 <= nx < sm.shape[1] and dist[ny, nx] > d:
                dist[ny, nx] = d
                q.append((ny, nx))
    near = dist * sres <= a.dilate_m

    lethal = cm == 0            # costmap は cost 100 -> 0 (黒) で書いてある
    ys, xs = np.nonzero(lethal)
    phantom = 0
    cells = []
    for y, x in zip(ys.tolist(), xs.tolist()):
        wx = (x + 0.5) * cres + cox
        wy = (cm.shape[0] - 1 - y + 0.5) * cres + coy
        sx = int((wx - sox) / sres)
        sy = sm.shape[0] - 1 - int((wy - soy) / sres)
        if not (0 <= sy < sm.shape[0] and 0 <= sx < sm.shape[1]) or not near[sy, sx]:
            phantom += 1
            cells.append((round(wx, 2), round(wy, 2)))
    print(f'costmap の致命セル {len(ys)}, うち静的な壁から {a.dilate_m} m 以上 '
          f'離れたもの (= 幻の障害物) {phantom} '
          f'({100*phantom/max(len(ys),1):.1f}%)')
    if cells:
        cx = sum(c[0] for c in cells) / len(cells)
        cy = sum(c[1] for c in cells) / len(cells)
        far = max(cells, key=lambda c: (c[0]-cx)**2 + (c[1]-cy)**2)
        print(f'  重心 ({cx:.1f}, {cy:.1f}), 例 {cells[:6]}, 最も遠い {far}')
        # 連結成分の数 (幻がいくつの塊になっているか)
        s = set((round(c[0]/cres), round(c[1]/cres)) for c in cells)
        seen, blobs = set(), 0
        for c in list(s):
            if c in seen:
                continue
            blobs += 1
            st = [c]
            seen.add(c)
            while st:
                p = st.pop()
                for d in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                    n = (p[0]+d[0], p[1]+d[1])
                    if n in s and n not in seen:
                        seen.add(n)
                        st.append(n)
        print(f'  塊の数 {blobs}')


if __name__ == '__main__':
    main()
