#!/usr/bin/env python3
"""
live_plot.py - live torque / speed / position / temperature of all 4 motors.

  python live_plot.py -p COM8            # last 10 s
  python live_plot.py -p COM8 --win 30 --log run1.csv

Motors stay in whatever mode they are in (this script only watches).
"""
import argparse
import math

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from rs05_host import RS05Link, fault_str, MODE_NAMES

COLORS = ["#e6261f", "#0a5cff", "#00a05a", "#ffb400"]
PANELS = [("Torque [Nm]", lambda m: m.torque),
          ("Speed [rpm]", lambda m: m.rpm),
          ("Position rel. zero [deg]", lambda m: m.pos_deg),
          ("Temperature [C]", lambda m: m.temp)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", required=True)
    ap.add_argument("--win", type=float, default=10.0, help="window [s]")
    ap.add_argument("--log", help="also record CSV")
    a = ap.parse_args()

    L = RS05Link(a.port, verbose_events=True)
    info = L.ping()
    L.set_cfg("TELEM_DIV", 5)   # 100 Hz
    if a.log:
        L.start_log(a.log)

    fig, axes = plt.subplots(2, 2, figsize=(12, 7), sharex=True)
    fig.canvas.manager.set_window_title(f"RS05 live - fw {info['fw']}")
    lines = []
    for ax, (title, _) in zip(axes.flat, PANELS):
        ax.set_title(title, loc="left", fontsize=11)
        ax.grid(alpha=0.3)
        lines.append([ax.plot([], [], color=c, lw=1.6, label=f"M{k+1}")[0] for k, c in enumerate(COLORS)])
    axes[0, 0].legend(loc="upper left", ncol=4, fontsize=9)
    for ax in axes[1]:
        ax.set_xlabel("t [s]")
    status = fig.text(0.01, 0.005, "", family="monospace", fontsize=9)

    def update(_):
        hist = list(L.history)
        if len(hist) < 2:
            return
        t_end = hist[-1].host_t
        hist = [h for h in hist if h.host_t >= t_end - a.win]
        t = [h.host_t - t_end for h in hist]
        for (title, f), ln, ax in zip(PANELS, lines, axes.flat):
            for k in range(4):
                ln[k].set_data(t, [f(h.m[k]) for h in hist])
            ax.relim()
            ax.autoscale_view(scalex=False)
            ax.set_xlim(-a.win, 0)
        last = hist[-1]
        status.set_text("   ".join(
            f"M{k+1}:{MODE_NAMES.get(m.mode,'?')}{'' if m.online else '(off)'} {m.torque:+.2f}Nm "
            f"{m.rpm:+.0f}rpm {m.pos_deg:+.1f}deg {m.temp:.0f}C flt:{fault_str(m.fault)}"
            for k, m in enumerate(last.m)))

    anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)  # noqa: F841
    try:
        plt.tight_layout(rect=(0, 0.03, 1, 1))
        plt.show()
    finally:
        L.close()


if __name__ == "__main__":
    main()
