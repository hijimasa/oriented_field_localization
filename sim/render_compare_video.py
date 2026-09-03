#!/usr/bin/env python3
"""複数の閉ループ走行 (run.csv) を同期再生する比較動画を描く。

    ./render_compare_video.py \
        --run "out_amcl_kidnap=AMCL (standard)" \
        --run "out_amcl_reinit_kidnap=AMCL + uniform re-scatter (oracle)" \
        --run "out_sup_ack_kidnap=AMCL + OFL supervision" \
        --out compare.mp4

各パネルに同じ地図・同じ時刻の真値 (黒) と推定 (色) を描き、下段に位置誤差の
時系列を流す。真値と推定を結ぶ赤い線が「いまどれだけ間違っているか」で、
この線が伸びたまま戻らないのが kidnap に負けた条件、すぐ縮むのが復帰できた
条件である。

速度は区間ごとに変えられる (--segments)。既定は kidnap の前後だけ 2 倍速、
それ以外は 6 倍速。
"""
import argparse
import bisect
import csv
import json
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import animation

TRAIL_S = 20.0        # 軌跡を残す長さ [s]
ERR_CLIP = 3.0        # 誤差グラフの上限 [m] (kidnap の 14 m はクリップして表示)
FLASH_S = 5.0         # KIDNAP / RE-SCATTER 表示を出す長さ [s]
COLORS = ["#e8590c", "#2f9e44", "#1971c2", "#9b7bff"]   # 最後の走行が青になる順で
PANEL_W = 5.6         # 1 パネルの幅 [inch]


def load_run(out_dir):
    rows = list(csv.DictReader(open(os.path.join(out_dir, "run.csv"))))
    run = {k: [] for k in ("t", "gx", "gy", "gyaw", "ex", "ey", "eyaw", "err")}
    for r in rows:
        try:
            v = [float(r[k]) for k in
                 ("t", "gt_x", "gt_y", "gt_yaw", "est_x", "est_y", "est_yaw", "pos_err")]
        except ValueError:
            continue
        for k, x in zip(("t", "gx", "gy", "gyaw", "ex", "ey", "eyaw", "err"), v):
            run[k].append(x)
    ev_path = os.path.join(out_dir, "run_events.json")
    events = []
    if os.path.exists(ev_path):
        ev = json.load(open(ev_path))
        events = ev["events"] if isinstance(ev, dict) else ev
    run["goals_done"] = sorted(
        e["t"] for e in events
        if e.get("ev") == "goal_done" and e.get("status") == 4)
    run["goals_sent"] = [(e["t"], e["xy"]) for e in events if e.get("ev") == "goal_sent"]
    run["reinit_t"] = next(
        (e["t"] for e in events if e.get("ev") == "reinitialize"), None)
    run["kidnap_t"] = None
    for r in rows:
        if r.get("kidnapped") == "1":
            run["kidnap_t"] = float(r["t"])
            break
    # >0.5 m の合計時間 (表示用)
    over = 0.0
    for i in range(1, len(run["t"])):
        if run["err"][i] > 0.5:
            over += run["t"][i] - run["t"][i - 1]
    run["over_s"] = over
    return run


def load_map(out_dir):
    yaml_path = os.path.join(out_dir, "env", "office.yaml")
    res, ox, oy, img_name = 0.05, 0.0, 0.0, "office.pgm"
    for line in open(yaml_path):
        if line.startswith("image:"):
            img_name = line.split(":", 1)[1].strip()
        elif line.startswith("resolution:"):
            res = float(line.split(":", 1)[1])
        elif line.startswith("origin:"):
            ox, oy = [float(v) for v in
                      line.split("[", 1)[1].split("]")[0].split(",")[:2]]
    img = plt.imread(os.path.join(out_dir, "env", img_name))
    h, w = img.shape[:2]
    return img, (ox, ox + w * res, oy, oy + h * res)


class Panel:
    """1 走行ぶんの地図パネル。"""

    def __init__(self, ax, run, map_img, extent, color, label):
        self.ax, self.run, self.color = ax, run, color
        ax.imshow(map_img, cmap="gray", extent=extent, origin="upper",
                  vmin=0, vmax=1 if map_img.dtype.kind == "f" else 255,
                  interpolation="nearest")
        ax.set_xlim(extent[0], extent[1])
        ax.set_ylim(extent[2], extent[3])
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_title(label, fontsize=12, color=color, fontweight="bold")
        for t, xy in run["goals_sent"]:
            ax.plot(xy[0], xy[1], "+", color="#bbbbbb", ms=6, mew=1, zorder=2)
        (self.gt_trail,) = ax.plot([], [], "-", color="#222222", lw=1.0,
                                   alpha=0.5, zorder=3)
        (self.est_trail,) = ax.plot([], [], "-", color=color, lw=1.4,
                                    alpha=0.7, zorder=4)
        (self.err_line,) = ax.plot([], [], "-", color="#e03131", lw=1.8, zorder=5)
        (self.gt_dot,) = ax.plot([], [], "o", color="#222222", ms=7, zorder=6)
        (self.gt_head,) = ax.plot([], [], "-", color="#222222", lw=2, zorder=6)
        (self.est_dot,) = ax.plot([], [], "o", color=color, ms=7, zorder=7)
        (self.est_head,) = ax.plot([], [], "-", color=color, lw=2, zorder=7)
        self.err_text = ax.text(0.02, 0.02, "", transform=ax.transAxes,
                                fontsize=11, va="bottom")
        self.kid_text = ax.text(0.5, 0.92, "", transform=ax.transAxes,
                                fontsize=14, ha="center", color="#e03131",
                                fontweight="bold")
        self.reinit_text = ax.text(0.5, 0.84, "", transform=ax.transAxes,
                                   fontsize=11, ha="center", color=color,
                                   fontweight="bold")

    def update(self, now):
        run = self.run
        i = bisect.bisect_right(run["t"], now) - 1
        if i < 0:
            return
        j = bisect.bisect_left(run["t"], now - TRAIL_S)
        self.gt_trail.set_data(run["gx"][j:i + 1], run["gy"][j:i + 1])
        self.est_trail.set_data(run["ex"][j:i + 1], run["ey"][j:i + 1])
        gx, gy, gyaw = run["gx"][i], run["gy"][i], run["gyaw"][i]
        ex, ey, eyaw = run["ex"][i], run["ey"][i], run["eyaw"][i]
        self.gt_dot.set_data([gx], [gy])
        self.gt_head.set_data([gx, gx + 0.45 * math.cos(gyaw)],
                              [gy, gy + 0.45 * math.sin(gyaw)])
        self.est_dot.set_data([ex], [ey])
        self.est_head.set_data([ex, ex + 0.45 * math.cos(eyaw)],
                               [ey, ey + 0.45 * math.sin(eyaw)])
        self.err_line.set_data([gx, ex], [gy, ey])
        err = run["err"][i]
        goals = sum(1 for t in run["goals_done"] if t <= now)
        self.err_text.set_text(f"error {err:.2f} m   goals {goals}")
        self.err_text.set_color("#e03131" if err > 0.5 else "#222222")
        kt = run["kidnap_t"]
        self.kid_text.set_text(
            "KIDNAP!" if kt is not None and kt <= now <= kt + FLASH_S else "")
        rt = run["reinit_t"]
        self.reinit_text.set_text(
            "uniform re-scatter" if rt is not None and rt <= now <= rt + FLASH_S
            else "")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", action="append", default=[],
                    help='"out_dir=Label" を並べる (2 つ以上)')
    ap.add_argument("--left")            # 2 パネル時代の呼び方 (互換)
    ap.add_argument("--right")
    ap.add_argument("--left-label", default="AMCL (standard)")
    ap.add_argument("--right-label", default="AMCL + OFL supervision")
    ap.add_argument("--out", default="compare.mp4")
    ap.add_argument("--fps", type=int, default=20)
    ap.add_argument("--segments", default=None,
                    help="comma-separated t0:t1:speed (default: 2x around the kidnap, 6x elsewhere)")
    args = ap.parse_args()

    specs = [tuple(r.split("=", 1)) for r in args.run]
    if args.left and args.right:
        specs = [(args.left, args.left_label), (args.right, args.right_label)]
    if len(specs) < 2:
        ap.error("give at least two runs (--run \"dir=Label\")")

    runs = [load_run(d) for d, _ in specs]
    labels = [l for _, l in specs]
    colors = COLORS[:len(specs) - 1] + [COLORS[2]]   # 最後 (提案手法) を青に
    if len(specs) == 2:
        colors = [COLORS[0], COLORS[2]]
    map_img, extent = load_map(specs[0][0])
    t_end = min(r["t"][-1] for r in runs)

    kt = next((r["kidnap_t"] for r in runs if r["kidnap_t"] is not None), None)
    if args.segments:
        segments = [tuple(float(v) for v in s.split(":"))
                    for s in args.segments.split(",")]
    elif kt is not None:
        segments = [(0, kt - 5, 6), (kt - 5, kt + 25, 2), (kt + 25, t_end, 6)]
    else:
        segments = [(0, t_end, 6)]

    # 各フレームのシミュレーション時刻 (区間ごとの速度で刻む)
    frame_ts = []
    for t0, t1, speed in segments:
        t = max(t0, frame_ts[-1] if frame_ts else 0.0)
        while t < min(t1, t_end):
            frame_ts.append(t)
            t += speed / args.fps
    frame_ts.append(t_end)

    n = len(specs)
    # libx264 (yuv420p) は偶数ピクセルしか受けない。インチ幅を整数に丸めると
    # 浮動小数として正確になり、100 dpi でちょうど偶数ピクセルになる
    w_in = float(math.ceil(PANEL_W * n + 0.6))
    fig = plt.figure(figsize=(w_in, 7.2), dpi=100)
    gs = fig.add_gridspec(2, n, height_ratios=[3.2, 1.0],
                          left=0.04, right=0.98, top=0.90, bottom=0.07,
                          wspace=0.05, hspace=0.16)
    panels = [Panel(fig.add_subplot(gs[0, k]), runs[k], map_img, extent,
                    colors[k], labels[k]) for k in range(n)]
    ax_e = fig.add_subplot(gs[1, :])

    # 下段: 位置誤差の時系列 (全体を薄く、経過分を濃く)
    progs = []
    for run, col, label in zip(runs, colors, labels):
        ax_e.plot(run["t"], [min(e, ERR_CLIP) for e in run["err"]],
                  color=col, lw=0.8, alpha=0.25)
        (p,) = ax_e.plot([], [], color=col, lw=1.6,
                         label=f"{label}  (>0.5 m: {run['over_s']:.1f} s)")
        progs.append(p)
    ax_e.axhline(0.5, color="#e03131", lw=0.8, ls="--")
    if kt is not None:
        ax_e.axvline(kt, color="#e03131", lw=0.8, ls=":")
        ax_e.text(kt, ERR_CLIP * 0.92, " kidnap", color="#e03131", fontsize=9)
    cursor = ax_e.axvline(0, color="#222222", lw=1.0)
    ax_e.set_xlim(0, t_end)
    ax_e.set_ylim(0, ERR_CLIP)
    ax_e.set_xlabel("time [s]", fontsize=10)
    ax_e.set_ylabel(f"position error [m]\n(clipped at {ERR_CLIP:.0f} m)", fontsize=9)
    ax_e.legend(loc="upper left", fontsize=9, framealpha=0.9,
                ncol=1 if n <= 2 else n)
    ax_e.tick_params(labelsize=9)

    title = fig.suptitle("", fontsize=13, x=0.45, ha="center")
    fig.text(0.99, 0.975, "black = ground truth\ncolor = estimate,  red = error",
             fontsize=9, color="#555555", ha="right", va="top")

    def draw(k):
        now = frame_ts[k]
        for panel in panels:
            panel.update(now)
        for run, p in zip(runs, progs):
            i = bisect.bisect_right(run["t"], now)
            p.set_data(run["t"][:i], [min(e, ERR_CLIP) for e in run["err"][:i]])
        cursor.set_xdata([now, now])
        speed = next((s for t0, t1, s in segments if t0 <= now < t1), segments[-1][2])
        title.set_text(f"Nav2 closed loop with a kidnap    t = {now:5.1f} s    x{speed:g}")

    anim = animation.FuncAnimation(fig, draw, frames=len(frame_ts))
    anim.save(args.out, writer=animation.FFMpegWriter(
        fps=args.fps, bitrate=3500, codec="libx264",
        extra_args=["-pix_fmt", "yuv420p"]))
    print(f"wrote {args.out}: {len(frame_ts)} frames, "
          f"{len(frame_ts) / args.fps:.0f} s of video for {t_end:.0f} s of sim")


if __name__ == "__main__":
    main()
