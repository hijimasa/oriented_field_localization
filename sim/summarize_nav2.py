#!/usr/bin/env python3
"""Nav2 閉ループ走行 (run.csv + run_events.json) を要約する。

開ループの要約 (summarize_run.py) に加えて、閉ループでしか出ない量を出す。

  - **到達**: 目標の成功数・所要時間。位置推定が壊れれば真っ先にここに出る。
  - **クリアランス**: 真値から見た壁・動的障害物までの最小距離。推定誤差は
    最終的に「壁にどれだけ近づいたか」に化ける。0.17 m (内接半径) を割ったら
    車体が重なっている = 接触。
  - **跳び**: map->odom の 1 ステップ変化。大域位置推定が制御へ直接入れる
    不連続で、AMCL のような逐次推定には原理的に出ない量。
"""
import csv
import json
import math
import os
import re
import statistics
import sys

INSCRIBED = 0.17     # 車体 0.40 x 0.34 の内接半径 [m]

ACCEPT = re.compile(
    r'(GLOBAL|TRACK) accept .* wfrac (-?[\d.]+) in ([\d.]+) ms')
REJECT = re.compile(r'(GLOBAL|TRACK) reject \((\w+) (-?[\d.]+)')


def localizer_log(path):
    """ofl_node のログから採択・棄却の内訳を出す。

    動的障害物のように「地図に無いものが視野に入る」条件で WFRAC ゲートが
    誤って棄却していないかは、ここでしか分からない。
    """
    if not os.path.exists(path):
        return
    acc = {'GLOBAL': 0, 'TRACK': 0}
    rej = {}
    wfr, ms = [], []
    for line in open(path, errors='ignore'):
        m = ACCEPT.search(line)
        if m:
            acc[m.group(1)] += 1
            w = float(m.group(2))
            if w >= 0:
                wfr.append(w)
            ms.append(float(m.group(3)))
            continue
        m = REJECT.search(line)
        if m:
            rej[(m.group(1), m.group(2))] = rej.get((m.group(1), m.group(2)), 0) + 1
    if not acc['GLOBAL'] and not acc['TRACK'] and not rej:
        return
    total = acc['GLOBAL'] + acc['TRACK'] + sum(rej.values())
    print(f'\n位置推定器の内訳 ({total} スキャン)')
    print(f'  採択 GLOBAL {acc["GLOBAL"]}  TRACK {acc["TRACK"]}')
    if rej:
        print('  棄却 ' + ', '.join(f'{k[0]} {k[1]} {v}'
                                    for k, v in sorted(rej.items())))
    else:
        print('  棄却 なし')
    if wfr:
        print(f'  採択時 WFRAC 中央 {statistics.median(wfr):.3f}  '
              f'95% {pct(wfr,0.95):.3f}  最大 {max(wfr):.3f}')
    if ms:
        print(f'  1 スキャンあたり {statistics.median(ms):.1f} ms (中央), '
              f'{max(ms):.1f} ms (最大)')


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


def block(label, rows):
    pe, ye = fcol(rows, 'pos_err'), fcol(rows, 'yaw_err')
    oe = fcol(rows, 'odom_err')
    if not pe:
        print(f'{label}: 推定値なし')
        return
    print(f'{label} ({len(rows)} サンプル)')
    print(f'  位置誤差   中央 {statistics.median(pe):.3f}  95% {pct(pe,0.95):.3f}  '
          f'最大 {max(pe):.3f} m')
    print(f'  角度誤差   中央 {statistics.median(ye):.2f}  最大 {max(ye):.2f} deg')
    print(f'  0.5 m 以内 {100*sum(1 for v in pe if v < 0.5)/len(pe):.1f}%')
    if oe:
        print(f'  参考 オドメトリ単独 中央 {statistics.median(oe):.3f}  最大 {max(oe):.3f} m')


def main(path):
    rows = list(csv.DictReader(open(path)))
    if not rows:
        raise SystemExit('空の CSV')
    ev_path = os.path.splitext(path)[0] + '_events.json'
    ev = json.load(open(ev_path)) if os.path.exists(ev_path) else {'events': []}

    kid = [r for r in rows if r['kidnapped'] == '1']
    kid_t = float(kid[0]['t']) if kid else None

    if kid_t is None:
        block('位置推定 (全区間)', rows)
    else:
        block('位置推定 (kidnap 前)', [r for r in rows if float(r['t']) < kid_t])
        after = [r for r in rows if float(r['t']) >= kid_t]
        block('位置推定 (kidnap 後)', after)
        rec, start = None, None
        for r in after:
            try:
                v = float(r['pos_err'])
            except (ValueError, TypeError):
                start = None
                continue
            if v < 0.5:
                if start is None:
                    start = float(r['t'])
                elif float(r['t']) - start >= 2.0:
                    rec = start
                    break
            else:
                start = None
        print(f'  kidnap ({kid_t:.1f}s) からの復帰: '
              + (f'{rec - kid_t:.1f} s' if rec is not None else '復帰せず'))

    # ---- 到達 ----
    sent = [e for e in ev['events'] if e['ev'] == 'goal_sent']
    done = [e for e in ev['events'] if e['ev'] == 'goal_done']
    fail = [e for e in ev['events'] if e['ev'] in ('goal_failed', 'goal_rejected')]
    print(f'\n到達  成功 {len(done)} / 送信 {len(sent)}  失敗 {len(fail)}')
    if done:
        dts = [e['dt'] for e in done]
        errs = [e['err'] for e in done if e.get('err') is not None]
        print(f'  1 目標あたり {statistics.median(dts):.1f} s (中央), 最大 {max(dts):.1f} s')
        if errs:
            print(f'  到達時の真の目標残差 中央 {statistics.median(errs):.3f} m  '
                  f'最大 {max(errs):.3f} m')
    if fail:
        print(f'  失敗の内訳: ' + ', '.join(
            f"t={e['t']:.0f}s goal{e['i']}(status {e.get('status','-')})" for e in fail[:8]))
    if ev.get('nav_ready_t') is not None:
        print(f'  位置推定 + Nav2 が揃うまで {ev["nav_ready_t"]:.1f} s')

    # ---- クリアランス ----
    wc = fcol(rows, 'wall_clr')
    oc = fcol(rows, 'obs_clr')
    if wc:
        print(f'\nクリアランス (真値基準)')
        print(f'  壁まで      中央 {statistics.median(wc):.2f}  5% {pct(wc,0.05):.2f}  '
              f'最小 {min(wc):.2f} m')
        n_hit = sum(1 for v in wc if v < INSCRIBED)
        print(f'  壁と重なった標本 {n_hit} / {len(wc)} '
              f'({100*n_hit/len(wc):.2f}%, 内接半径 {INSCRIBED} m)')
    if oc:
        print(f'  動的障害物まで 中央 {statistics.median(oc):.2f}  5% {pct(oc,0.05):.2f}  '
              f'最小 {min(oc):.2f} m')
        n_hit = sum(1 for v in oc if v < INSCRIBED)
        print(f'  障害物と重なった標本 {n_hit} / {len(oc)} ({100*n_hit/len(oc):.2f}%)')

    # ---- 跳び ----
    # kidnap の瞬間は真値が飛ぶので、そこは通常運用の不連続とは別に扱う
    # (混ぜると kidnap 条件だけ最大値が跳ね上がって比較にならない)
    calm = [r for r in rows if kid_t is None or float(r['t']) < kid_t]
    ej = fcol(calm, 'err_jump_m')
    ejd = fcol(calm, 'err_jump_deg')
    jm = fcol(calm, 'tf_jump_m')
    if ej:
        big = [v for v in ej if v > 0.05]
        print(f'\n制御へ入る不連続 (推定誤差ベクトルの 1 ステップ変化, 50 ms'
              + (', kidnap 前' if kid_t is not None else '') + ')')
        print(f'  中央 {statistics.median(ej):.4f} m  95% {pct(ej,0.95):.4f} m  '
              f'最大 {max(ej):.3f} m / {max(ejd):.1f} deg')
        print(f'  0.05 m を超えた回数 {len(big)} / {len(ej)} '
              f'({100*len(big)/len(ej):.2f}%)')
    if jm:
        print(f'  参考 map->odom の変化 95% {pct(jm,0.95):.4f} m  最大 {max(jm):.3f} m'
              f' (オドメトリのスリップも含む)')
    if kid_t is not None:
        post = fcol([r for r in rows if float(r['t']) >= kid_t], 'err_jump_m')
        if post:
            print(f'  kidnap 後の最大の補正 {max(post):.2f} m '
                  f'(瞬間移動そのものを含む)')

    localizer_log(os.path.join(os.path.dirname(os.path.abspath(path)), 'loc.log'))

    # ---- 走行 ----
    dist = 0.0
    for a, b in zip(rows[:-1], rows[1:]):
        try:
            dist += math.hypot(float(b['gt_x']) - float(a['gt_x']),
                               float(b['gt_y']) - float(a['gt_y']))
        except (ValueError, TypeError):
            pass
    v = fcol(rows, 'cmd_v')
    moving = 100 * sum(1 for x in v if abs(x) > 0.02) / len(v) if v else 0.0
    wall = fcol(rows, 'wall')
    rtf = (float(rows[-1]['t']) / wall[-1]) if wall and wall[-1] > 0 else float('nan')
    print(f'\n走行 {float(rows[-1]["t"]):.1f} s, {dist:.1f} m, '
          f'前進していた時間 {moving:.0f}%, 実時間倍率 {rtf:.2f}x')


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'out_nav2/run.csv')
