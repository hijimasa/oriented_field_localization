#!/usr/bin/env python3
"""連続走行の記録 (run.csv) を要約する。

位置推定の誤差は「最新の推定と、その時点の真値」の差である。位置推定は
スキャンごとにしか更新されないので、更新の間は前回値との差が乗る。
それも含めた運用上の誤差として扱う。

odom_err は「オドメトリだけを積分した場合の誤差」で、位置推定が何を
直しているかの基準になる。
"""
import csv
import math
import statistics
import sys


def main(path):
    rows = list(csv.DictReader(open(path)))
    if not rows:
        raise SystemExit('空の CSV')

    def col(name):
        out = []
        for r in rows:
            try:
                v = float(r[name])
                if v == v:
                    out.append(v)
            except (ValueError, TypeError):
                pass
        return out

    t = col('t')
    duration = max(t) if t else 0.0
    kid_rows = [r for r in rows if r['kidnapped'] == '1']
    kid_t = float(kid_rows[0]['t']) if kid_rows else None

    def report(label, sub):
        pe = [float(r['pos_err']) for r in sub
              if r['pos_err'] not in ('', 'nan')]
        ye = [float(r['yaw_err']) for r in sub
              if r['yaw_err'] not in ('', 'nan')]
        oe = [float(r['odom_err']) for r in sub
              if r['odom_err'] not in ('', 'nan')]
        if not pe:
            print(f'{label}: 推定値なし')
            return
        pe_s = sorted(pe)
        good = 100 * sum(1 for v in pe if v < 0.5) / len(pe)
        print(f'{label} ({len(sub)} サンプル, {sub[-1]["t"]}s まで)')
        print(f'  位置誤差   中央 {statistics.median(pe):.3f} m  '
              f'95% {pe_s[int(0.95 * (len(pe_s) - 1))]:.3f} m  最大 {max(pe):.3f} m')
        print(f'  角度誤差   中央 {statistics.median(ye):.2f} deg  最大 {max(ye):.2f} deg')
        print(f'  0.5 m 以内 {good:.1f}%')
        if oe:
            print(f'  参考: オドメトリ単独の誤差 中央 {statistics.median(oe):.3f} m  '
                  f'最大 {max(oe):.3f} m')

    if kid_t is None:
        report('全区間', rows)
    else:
        report('kidnap 前', [r for r in rows if float(r['t']) < kid_t])
        after = [r for r in rows if float(r['t']) >= kid_t]
        report('kidnap 後 (全体)', after)
        # 復帰時刻: kidnap 後に位置誤差が 0.5 m を下回って 2 秒続いた最初の時刻
        rec = None
        run_start = None
        for r in after:
            try:
                v = float(r['pos_err'])
            except (ValueError, TypeError):
                run_start = None
                continue
            if v < 0.5:
                if run_start is None:
                    run_start = float(r['t'])
                elif float(r['t']) - run_start >= 2.0:
                    rec = run_start
                    break
            else:
                run_start = None
        if rec is not None:
            print(f'  kidnap ({kid_t:.1f}s) から復帰まで {rec - kid_t:.1f} s')
        else:
            print(f'  kidnap ({kid_t:.1f}s) から復帰しなかった')

    # 走行距離
    dist = 0.0
    for a, b in zip(rows[:-1], rows[1:]):
        try:
            dist += math.hypot(float(b['gt_x']) - float(a['gt_x']),
                               float(b['gt_y']) - float(a['gt_y']))
        except (ValueError, TypeError):
            pass
    print(f'\n走行: {duration:.1f} s, {dist:.1f} m')


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'out/run.csv')
