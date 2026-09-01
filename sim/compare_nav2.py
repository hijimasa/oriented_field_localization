#!/usr/bin/env python3
"""複数の Nav2 閉ループ走行を 1 枚の表にまとめる。

usage: compare_nav2.py <label>=<run_dir> [...]
"""
import csv
import json
import math
import os
import statistics
import sys

INSCRIBED = 0.17


def fcol(rows, name):
    out = []
    for r in rows:
        try:
            v = float(r[name])
            if v == v:
                out.append(v)
        except (ValueError, TypeError, KeyError):
            pass
    return out


def pct(v, p):
    s = sorted(v)
    return s[int(p * (len(s) - 1))] if s else float('nan')


def load(d):
    rows = list(csv.DictReader(open(os.path.join(d, 'run.csv'))))
    ev_path = os.path.join(d, 'run_events.json')
    ev = json.load(open(ev_path)) if os.path.exists(ev_path) else {'events': []}
    return rows, ev


def stats(rows, ev):
    kid = [r for r in rows if r.get('kidnapped') == '1']
    kid_t = float(kid[0]['t']) if kid else None
    calm = [r for r in rows if kid_t is None or float(r['t']) < kid_t]
    pe = fcol(rows, 'pos_err')
    wc = fcol(rows, 'wall_clr')
    oc = fcol(rows, 'obs_clr')
    jm = fcol(calm, 'err_jump_m')
    wall = fcol(rows, 'wall')
    sent = sum(1 for e in ev['events'] if e['ev'] == 'goal_sent')
    done = [e for e in ev['events'] if e['ev'] == 'goal_done']
    fail = sum(1 for e in ev['events'] if e['ev'] in ('goal_failed', 'goal_rejected'))
    dist = 0.0
    for a, b in zip(rows[:-1], rows[1:]):
        try:
            dist += math.hypot(float(b['gt_x']) - float(a['gt_x']),
                               float(b['gt_y']) - float(a['gt_y']))
        except (ValueError, TypeError):
            pass
    t_end = float(rows[-1]['t'])
    return {
        'goals': len(done),
        'fail': fail,
        'goal_s': statistics.median([e['dt'] for e in done]) if done else float('nan'),
        'med': statistics.median(pe) if pe else float('nan'),
        'p95': pct(pe, 0.95),
        'max': max(pe) if pe else float('nan'),
        'in05': 100 * sum(1 for v in pe if v < 0.5) / len(pe) if pe else float('nan'),
        'wall5': pct(wc, 0.05), 'wallmin': min(wc) if wc else float('nan'),
        'obsmin': min(oc) if oc else float('nan'),
        'contact': sum(1 for v in wc if v < INSCRIBED)
                   + sum(1 for v in oc if v < INSCRIBED),
        'jump95': pct(jm, 0.95), 'jumpmax': max(jm) if jm else float('nan'),
        'dist': dist, 't': t_end,
        'rtf': t_end / wall[-1] if wall and wall[-1] > 0 else float('nan'),
    }


def main(args):
    runs = []
    for a in args:
        label, d = a.split('=', 1)
        if not os.path.exists(os.path.join(d, 'run.csv')):
            print(f'!! {label}: run.csv が無い ({d})', file=sys.stderr)
            continue
        runs.append((label, stats(*load(d))))

    def row(cells):
        print('| ' + ' | '.join(cells) + ' |')

    print('### 到達と位置推定\n')
    row(['条件', '到達', '失敗', '1目標 [s]', '位置誤差 中央 [m]',
         '95% [m]', '最大 [m]', '0.5 m 以内'])
    row(['---'] * 8)
    for label, s in runs:
        row([label, str(s['goals']), str(s['fail']), f"{s['goal_s']:.1f}",
             f"{s['med']:.3f}", f"{s['p95']:.3f}", f"{s['max']:.3f}",
             f"{s['in05']:.1f}%"])

    print('\n### 余裕と跳び\n')
    row(['条件', '壁まで 5% [m]', '壁まで 最小 [m]', '障害物まで 最小 [m]',
         '接触標本', '不連続 95% [m]', '最大 [m]', '走行 [m]', 'RTF'])
    row(['---'] * 9)
    for label, s in runs:
        om = '-' if s['obsmin'] != s['obsmin'] else f"{s['obsmin']:.2f}"
        row([label, f"{s['wall5']:.2f}", f"{s['wallmin']:.2f}", om,
             str(s['contact']), f"{s['jump95']:.3f}", f"{s['jumpmax']:.3f}",
             f"{s['dist']:.1f}", f"{s['rtf']:.2f}x"])


if __name__ == '__main__':
    main(sys.argv[1:])
