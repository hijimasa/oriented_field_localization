#!/usr/bin/env python3
"""reseed_margin_eval の出力を集計し、監視判定のしきい値を較正する。

    ./summarize_reseed.py reseed_margin.csv

per-scan の完全ルール (幾何: wfrac_other > max_wfrac かつ excess > wfrac_excess、
不一致: 距離 > 0.5 m または > 20 deg、の AND) で発火率を出す。
実機ではこの上に連続 10 スキャンと最小間隔の鈍化が乗るので、ここの値は
per-scan の上界である。

見るもの:
  1. クラス別の発火率。healthy は 0 でなければならない。mislock / kidnap は
     高いほどよい。drifted は境界帯 (どこから発火が始まるかを曲線で見る)
  2. 判定を 2 段に分解した寄与 (幾何だけ / 不一致だけ / 両方)
  3. 条件別の発火率 (どの外乱が誤発火・取り逃しに近いか)
  4. しきい値の掃引 (どこまで動かすと誤発火が出るか、検出がどこで落ちるか)
"""
import csv
import sys
from collections import defaultdict

MAX_WFRAC = 0.35        # supervise_max_wfrac の既定
EXCESS = 0.15           # supervise_wfrac_excess の既定
DISAGREE_M = 0.5        # supervise_min_disagreement_m の既定
DISAGREE_DEG = 20.0     # supervise_min_disagreement_deg の既定


def geometry_bad(row, max_wfrac=MAX_WFRAC, excess=EXCESS):
    return row["wfrac_other"] > max_wfrac and row["excess"] > excess


def poses_disagree(row):
    return row["d_m"] > DISAGREE_M or row["d_deg"] > DISAGREE_DEG


def fires(row, max_wfrac=MAX_WFRAC, excess=EXCESS):
    return geometry_bad(row, max_wfrac, excess) and poses_disagree(row)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "out/reseed_margin.csv"
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append({
                "cond": r["cond"], "kind": r["kind"],
                "d_m": float(r["d_m"]), "d_deg": float(r["d_deg"]),
                "wfrac_other": float(r["wfrac_other"]),
                "wfrac_self": float(r["wfrac_self"]),
                "excess": float(r["excess"]),
            })
    if not rows:
        print("no rows", file=sys.stderr)
        return 1

    kinds = ["healthy", "drifted", "mislock", "kidnap"]
    by_kind = defaultdict(list)
    for r in rows:
        by_kind[r["kind"]].append(r)

    def rate(rs, pred):
        return 100.0 * sum(pred(r) for r in rs) / len(rs) if rs else float("nan")

    print(f"== per-scan の完全ルール (max_wfrac {MAX_WFRAC} / excess {EXCESS} / "
          f"不一致 {DISAGREE_M} m or {DISAGREE_DEG} deg) ==")
    print(f"{'class':>8}  {'n':>5}  {'発火%':>6}  {'幾何のみ%':>8}  {'不一致のみ%':>9}")
    for k in kinds:
        rs = by_kind[k]
        if not rs:
            continue
        print(f"{k:>8}  {len(rs):>5}  {rate(rs, fires):>5.1f}%"
              f"  {rate(rs, geometry_bad):>7.1f}%  {rate(rs, poses_disagree):>8.1f}%")

    print("\n== 発火が始まる距離 (healthy + drifted, |dyaw| <= 25 deg, 0.1 m 刻み) ==")
    curve = [r for r in rows if r["kind"] in ("healthy", "drifted") and r["d_deg"] <= 25.0]
    print(f"{'d [m]':>10}  {'n':>5}  {'発火%':>6}  {'幾何%':>6}")
    for lo10 in range(0, 16):
        lo, hi = lo10 / 10.0, (lo10 + 1) / 10.0
        rs = [r for r in curve if lo <= r["d_m"] < hi]
        if not rs:
            continue
        print(f"{lo:>4.1f}--{hi:<4.1f}  {len(rs):>5}  {rate(rs, fires):>5.1f}%"
              f"  {rate(rs, geometry_bad):>5.1f}%")

    print("\n== 条件別の発火率 (完全ルール, healthy は誤発火なので 0 のこと) ==")
    conds = sorted({r["cond"] for r in rows})
    print(f"{'cond':>12}  " + "  ".join(f"{k:>8}" for k in kinds))
    for c in conds:
        cells = []
        for k in kinds:
            rs = [r for r in by_kind[k] if r["cond"] == c]
            cells.append(f"{rate(rs, fires):>7.1f}%" if rs else f"{'-':>8}")
        print(f"{c:>12}  " + "  ".join(cells))

    print("\n== しきい値の掃引 (完全ルール: healthy 誤発火% / mislock 検出% / kidnap 検出%) ==")
    sweeps_a = [0.25, 0.30, 0.35, 0.40, 0.45, 0.50]
    sweeps_e = [0.05, 0.10, 0.15, 0.20, 0.25, 0.30]
    print(f"{'max_wfrac':>9} \\ excess " + "  ".join(f"{e:>14.2f}" for e in sweeps_e))
    for a in sweeps_a:
        cells = []
        for e in sweeps_e:
            vals = [rate(by_kind[k], lambda r: fires(r, a, e))
                    for k in ("healthy", "mislock", "kidnap")]
            cells.append(f"{vals[0]:>3.0f}/{vals[1]:>3.0f}/{vals[2]:>3.0f}%")
        print(f"{a:>9.2f}          " + "  ".join(f"{c:>14}" for c in cells))

    n_healthy_fire = sum(fires(r) for r in by_kind["healthy"])
    print(f"\nhealthy の誤発火 (既定): {n_healthy_fire} / {len(by_kind['healthy'])}")
    return 0 if n_healthy_fire == 0 else 2


if __name__ == "__main__":
    sys.exit(main())
