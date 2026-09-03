#!/usr/bin/env python3
"""ofl_eval / bbs_eval の CSV から条件別の成功率表を出す。

成功の定義は両者で共通: 位置誤差 < 1.0 m かつ 角度誤差 < 15 度。
複数の CSV を渡すと同じ表に横並びで出す (同一スキャンで走らせた場合のみ
対応比較として意味がある。run_compare.sh がそれを保証する)。

usage:
    summarize.py ofl=out/ofl.csv bbs=out/bbs.csv
    summarize.py out/custom.csv                    # ラベル省略時はファイル名
"""
import csv
import sys
from collections import Counter

POS_TOL_M = 1.0
ANG_TOL_DEG = 15.0

# 列名は評価器ごとに違う。(位置誤差, 角度誤差) の候補を順に探す。
COLUMN_SETS = [
    ("pos_err", "ang_err"),
    ("wfrac_gate_pos_err", "wfrac_gate_ang_err"),
]


def load(path, pos_key="pos_err", ang_key="ang_err"):
    rows = list(csv.DictReader(open(path)))
    if not rows:
        raise SystemExit(f"{path}: 空の CSV")
    if pos_key not in rows[0]:
        raise SystemExit(f"{path}: 列 {pos_key} が無い (列: {', '.join(rows[0])})")
    return rows


def success(row, pos_key, ang_key):
    try:
        return float(row[pos_key]) < POS_TOL_M and float(row[ang_key]) < ANG_TOL_DEG
    except (ValueError, TypeError):
        return False   # nan / 空 = 探索失敗


def main(argv):
    if not argv:
        raise SystemExit(__doc__)
    series = []
    for arg in argv:
        label, _, path = arg.partition("=")
        if not path:
            path, label = label, label.rsplit("/", 1)[-1].replace(".csv", "")
        rows = load(path)
        # WFRAC ゲート列があれば併せて出す
        variants = [(label, "pos_err", "ang_err")]
        if "wfrac_gate_pos_err" in rows[0]:
            variants.append((label + "+wfrac", "wfrac_gate_pos_err", "wfrac_gate_ang_err"))
        for name, pk, ak in variants:
            series.append((name, rows, pk, ak))

    conds = []
    for _, rows, _, _ in series:
        for r in rows:
            if r["cond"] not in conds:
                conds.append(r["cond"])

    names = [s[0] for s in series]
    print(f"成功判定: 位置誤差 < {POS_TOL_M} m かつ 角度誤差 < {ANG_TOL_DEG} 度")
    print()
    print(f"{'条件':<12}{'n':>5}" + "".join(f"{n:>14}" for n in names))
    totals = Counter()
    counts = Counter()
    for c in conds:
        cells = []
        n_c = 0
        for name, rows, pk, ak in series:
            sub = [r for r in rows if r["cond"] == c]
            n_c = max(n_c, len(sub))
            hit = sum(success(r, pk, ak) for r in sub)
            cells.append(f"{100 * hit / len(sub):>13.1f}%" if sub else f"{'-':>14}")
            totals[name] += hit
            counts[name] += len(sub)
        print(f"{c:<12}{n_c:>5}" + "".join(cells))
    print(f"{'全体':<12}{max(counts.values()):>5}"
          + "".join(f"{100 * totals[n] / counts[n]:>13.1f}%" for n in names))


if __name__ == "__main__":
    main(sys.argv[1:])
