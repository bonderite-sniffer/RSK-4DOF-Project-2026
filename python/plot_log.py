#!/usr/bin/env python3
"""Plot a log from rs05_tests.py.

  python plot_log.py logs/pos_20261111_170000.csv
  python plot_log.py logs/velsweep_..._summary.csv      # torque/current/temp vs rpm
  python plot_log.py logs/dyno_..._points.csv           # measured T-N curve
"""
import math
import sys

import matplotlib.pyplot as plt
import pandas as pd


def plot_timeseries(df, title):
    t = (df["t_ms"] - df["t_ms"].iloc[0]) / 1000.0
    fig, ax = plt.subplots(5, 1, sharex=True, figsize=(11, 10))
    for m, style in ((1, "-"), (2, "--"), (3, "-."), (4, ":")):
        if df[f"m{m}_flags"].max() == 0:
            continue
        ax[0].plot(t, df[f"m{m}_pos"].apply(math.degrees), style, label=f"M{m} pos")
        sp_pos = df[f"m{m}_mode"] == 3
        ax[0].plot(t[sp_pos], df[f"m{m}_sp"][sp_pos].apply(math.degrees), ":", label=f"M{m} target")
        ax[1].plot(t, df[f"m{m}_vel"] * 60 / (2 * math.pi), style, label=f"M{m}")
        ax[2].plot(t, df[f"m{m}_torque"], style, label=f"M{m}")
        ax[3].plot(t, df[f"m{m}_iq"], style, label=f"M{m}")
        ax[4].plot(t, df[f"m{m}_temp"], style, label=f"M{m}")
    for a, lab in zip(ax, ["pos [deg]", "vel [rpm]", "torque [Nm]", "iq [A]", "temp [C]"]):
        a.set_ylabel(lab)
        a.grid(alpha=0.3)
        a.legend(loc="upper right", fontsize=8)
    ax[-1].set_xlabel("time [s]")
    fig.suptitle(title)
    fig.tight_layout()


def plot_summary(df, title):
    fig, ax = plt.subplots(1, 3, figsize=(14, 4))
    for m, g in df.groupby("motor"):
        ax[0].errorbar(g["rpm_meas"], g["torque"], yerr=g["torque_sd"], fmt="o-", label=f"M{m}")
        ax[1].plot(g["rpm_meas"], g["iq_absmax"], "o-", label=f"M{m}")
        ax[2].plot(g["rpm_meas"], g["temp_max"], "o-", label=f"M{m}")
    for a, lab in zip(ax, ["mean torque [Nm]", "max |iq| [A]", "max temp [C]"]):
        a.set_xlabel("speed [rpm]")
        a.set_ylabel(lab)
        a.grid(alpha=0.3)
        a.legend()
    fig.suptitle(title)
    fig.tight_layout()


def plot_dyno(df, title):
    fig, ax = plt.subplots(figsize=(7, 5))
    ok = df[df["limit"] == 0]
    edge = df.groupby("rpm_cmd").apply(lambda g: g[g["limit"] == 0]["m1_torque"].abs().max())
    ax.scatter(ok["rpm_meas"], ok["m1_torque"].abs(), s=10, alpha=0.4, label="held points")
    ax.plot(edge.index, edge.values, "o-", label="max torque held")
    ax.set_xlabel("speed [rpm]")
    ax.set_ylabel("torque [Nm]")
    ax.set_title(title + "  (compare with datasheet T-N @48V)")
    ax.grid(alpha=0.3)
    ax.legend()


if __name__ == "__main__":
    path = sys.argv[1]
    df = pd.read_csv(path)
    if "rpm_cmd" in df.columns and "load_cmd" in df.columns:
        plot_dyno(df, path)
    elif "rpm_cmd" in df.columns:
        plot_summary(df, path)
    else:
        plot_timeseries(df, path)
    plt.show()
