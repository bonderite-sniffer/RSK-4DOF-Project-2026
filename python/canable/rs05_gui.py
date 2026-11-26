#!/usr/bin/env python3
"""rs05_gui.py - RS05 test bench GUI over a CANable v2 (one motor).

    python rs05_gui.py              real motor (pick the COM port in the window)
    python rs05_gui.py --sim        simulated motor, to learn the GUI without hardware

Tabs: 1 Setup (zero + joint limits) | 2 Position | 3 Velocity | 4 MIT / Impedance | 5 Friction
STOP button / Esc key stops the motor from anywhere.
Joint limits are taught by hand in tab 1 and enforced in every mode (see rs05_controller.py).
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

import can
import matplotlib
from serial.tools import list_ports

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402

from rs05_can import (KD_MAX, MODELS, RS05, STATE_NAMES, deep_scan, deep_scan_bitrates, fault_str,  # noqa: E402
                      open_bus)
from rs05_controller import (FRICTION_DRIVES, LOOP_GAINS, STEP_MODES, Controller,  # noqa: E402
                             FrictionParams, FrictionTest, SafetyTrip, StepParams, StepTest, load_config,
                             move_config, read_loop_gains, save_config, write_loop_gains)

RPM = 60 / (2 * math.pi)
LOG_DIR = Path(__file__).resolve().parent / "logs"
PLOT_WINDOW_S = 10.0
# slcan / gs_usb = CANable straight to the motor (legacy, unchanged); stm = through the STM32 gateway
# firmware (rs05_hal/Firmware_Gateway, Port = the STM's USB COM port); sim / stm-sim = simulated motor,
# directly or behind a simulated STM.
IFACES = ("slcan", "gs_usb", "stm", "sim", "stm-sim")
ONE_BITRATE = ("sim", "stm", "stm-sim")     # no "all bitrates" scan through these
VEL_FILTER_N = 5            # median over 5 feedback frames (25-50 ms): removes the +-0.085 rad/s
                            # one-encoder-count spikes of the motor's speed estimate, keeps real motion
MIN_SPAN = {"p": 2.0, "v": 0.5, "t": 0.2}   # deg, rad/s, Nm/A: smallest y-range of each plot so a
                                            # motor at rest does not get zoomed into its LSB noise


def median_filter(xs, n=VEL_FILTER_N):
    """Centered running median (same length as xs)."""
    h, out = n // 2, []
    for i in range(len(xs)):
        w = sorted(xs[max(0, i - h):i + h + 1])
        out.append(w[len(w) // 2])
    return out


def autoscale_min_span(ax, ys, span):
    ys = [y for y in ys if y == y]          # drop NaN
    if not ys:
        return
    lo, hi = min(ys), max(ys)
    mid, half = (lo + hi) / 2, max((hi - lo) / 2 * 1.1, span / 2)
    ax.set_ylim(mid - half, mid + half)
SIM_CHANNEL = "rs05sim"

MIT_PRESETS = [  # name, kp, kd
    ("Transparent", 0.0, 0.02),
    ("Damping only", 0.0, 0.3),
    ("Soft spring", 3.0, 0.15),
    ("Medium", 10.0, 0.5),
    ("Stiff", 40.0, 1.0),
]


def fnum(var: tk.Variable, default: float = 0.0) -> float:
    try:
        return float(var.get())
    except (tk.TclError, ValueError):
        return default


class App(tk.Tk):
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.title("RS05 bench - CANable")
        w = min(1440, self.winfo_screenwidth() - 20)
        h = min(920, self.winfo_screenheight() - 80)
        self.geometry(f"{w}x{h}+0+0")
        self.minsize(1100, 700)
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.bind_all("<Escape>", lambda e: self.stop_motor())

        self.bus = None
        self.m: RS05 | None = None
        self.c: Controller | None = None
        self.sim = None
        self.stm_emu = None
        self.ft: FrictionTest | None = None
        self.ft_thread = None
        self.st: StepTest | None = None
        self._gains_read: dict = {}
        self.rec_file = None
        self._last_lim = None
        self._plot_tick = 0
        self._events_shown = 0

        style = ttk.Style(self)
        style.configure("Big.TLabel", font=("Segoe UI", 15, "bold"))
        style.configure("Hdr.TLabel", font=("Segoe UI", 10, "bold"))
        style.configure("Note.TLabel", foreground="#555")
        style.configure("Warn.TLabel", foreground="#b00020", font=("Segoe UI", 10, "bold"))

        self._build_top()
        body = ttk.Frame(self)
        body.pack(fill="both", expand=True, padx=6, pady=4)
        left = ttk.Frame(body, width=340)
        left.pack(side="left", fill="y")
        left.pack_propagate(False)
        right = ttk.Frame(body)
        right.pack(side="left", fill="both", expand=True, padx=(6, 0))
        self._build_left(left)
        self.nb = ttk.Notebook(right)
        self.nb.pack(fill="x")
        self._build_setup_tab()
        self._build_pos_tab()
        self._build_vel_tab()
        self._build_mit_tab()
        self._build_friction_tab()
        self._build_tune_tab()
        self._build_plot(right)
        self.after(100, self.refresh)

    # ================================================================== top bar
    def _build_top(self):
        f = ttk.Frame(self, padding=(6, 6, 6, 0))
        f.pack(fill="x")
        ttk.Label(f, text="Interface").pack(side="left")
        self.v_iface = tk.StringVar(value="sim" if self.args.sim else "slcan")
        ttk.Combobox(f, textvariable=self.v_iface, values=IFACES, width=8,
                     state="readonly").pack(side="left", padx=4)
        ttk.Label(f, text="Port").pack(side="left")
        self.v_port = tk.StringVar(value=self.args.port)
        self.cb_port = ttk.Combobox(f, textvariable=self.v_port, width=9)
        self.cb_port.pack(side="left", padx=4)
        ttk.Button(f, text="↻", width=3, command=self.refresh_ports).pack(side="left")
        ttk.Label(f, text="Bitrate").pack(side="left", padx=(10, 0))
        self.v_bitrate = tk.StringVar(value="1000000")
        ttk.Combobox(f, textvariable=self.v_bitrate, values=("1000000", "500000", "250000", "125000"),
                     width=9).pack(side="left", padx=4)
        ttk.Label(f, text="Motor ID").pack(side="left", padx=(10, 0))
        self.v_id = tk.StringVar(value=hex(self.args.id))
        self.cb_id = ttk.Combobox(f, textvariable=self.v_id, width=6)
        self.cb_id.pack(side="left", padx=4)
        ttk.Label(f, text="Host ID").pack(side="left")
        self.v_host = tk.StringVar(value=hex(self.args.host))
        ttk.Entry(f, textvariable=self.v_host, width=6).pack(side="left", padx=4)
        self.b_conn = ttk.Button(f, text="Connect", command=self.toggle_connect)
        self.b_conn.pack(side="left", padx=(10, 4))
        ttk.Button(f, text="Scan IDs", command=self.scan_ids).pack(side="left")
        self.l_conn = ttk.Label(f, text="disconnected", style="Note.TLabel")
        self.l_conn.pack(side="left", padx=10)
        self.refresh_ports()

    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.cb_port["values"] = ports
        if ports and self.v_port.get() not in ports and not self.args.sim:
            self.v_port.set(ports[0])

    # ================================================================== left column
    def _build_left(self, f):
        self.b_stop = tk.Button(f, text="STOP  (Esc)", bg="#c62828", fg="white", activebackground="#8e0000",
                                font=("Segoe UI", 18, "bold"), height=2, command=self.stop_motor)
        self.b_stop.pack(fill="x", pady=(0, 6))
        row = ttk.Frame(f)
        row.pack(fill="x")
        ttk.Button(row, text="Clear faults", command=lambda: self.call(lambda: self.c.clear_faults())).pack(
            side="left", expand=True, fill="x")
        self.v_rec = tk.BooleanVar(value=False)
        ttk.Checkbutton(row, text="Record CSV", variable=self.v_rec, command=self.toggle_record).pack(
            side="left", padx=6)

        self.l_mode = ttk.Label(f, text="MODE: -", style="Big.TLabel")
        self.l_mode.pack(anchor="w", pady=(8, 0))
        self.l_trip = ttk.Label(f, text="", style="Warn.TLabel", wraplength=330)
        self.l_trip.pack(anchor="w")

        grid = ttk.Frame(f)
        grid.pack(fill="x", pady=4)
        self.ro = {}
        for r, (key, label) in enumerate([("pos", "Position"), ("vel", "Velocity"), ("rpm", ""),
                                          ("tq", "Torque"), ("iq", "Current iq"), ("vbus", "VBUS"),
                                          ("temp", "Temp"), ("state", "Motor state"), ("flt", "Faults"),
                                          ("hz", "Feedback")]):
            ttk.Label(grid, text=label).grid(row=r, column=0, sticky="w")
            lab = ttk.Label(grid, text="-", style="Big.TLabel" if key in ("pos", "vel", "tq") else "TLabel")
            lab.grid(row=r, column=1, sticky="e", padx=4)
            self.ro[key] = lab
        grid.columnconfigure(1, weight=1)

        ttk.Label(f, text="Joint range", style="Hdr.TLabel").pack(anchor="w", pady=(8, 0))
        self.range_cv = tk.Canvas(f, height=70, bg="white", highlightthickness=1, highlightbackground="#ccc")
        self.range_cv.pack(fill="x")
        self.l_limits = ttk.Label(f, text="limits: not taught", style="Warn.TLabel")
        self.l_limits.pack(anchor="w")

        ttk.Label(f, text="Events", style="Hdr.TLabel").pack(anchor="w", pady=(8, 0))
        self.lb_events = tk.Listbox(f, height=12, font=("Consolas", 8))
        self.lb_events.pack(fill="both", expand=True)

    # ================================================================== tabs
    def _tab(self, title):
        fr = ttk.Frame(self.nb, padding=10)
        self.nb.add(fr, text=title)
        return fr

    def _entry(self, parent, label, value, row, col=0, width=8, unit=""):
        ttk.Label(parent, text=label).grid(row=row, column=col, sticky="w", padx=(0, 4), pady=2)
        var = tk.StringVar(value=str(value))
        ttk.Entry(parent, textvariable=var, width=width).grid(row=row, column=col + 1, sticky="w")
        if unit:
            ttk.Label(parent, text=unit, style="Note.TLabel").grid(row=row, column=col + 2, sticky="w", padx=4)
        return var

    # ------------------------------------------------------------------ 1 setup
    def _build_setup_tab(self):
        f = self._tab(" 1  Setup: zero & limits ")
        steps = ttk.Frame(f)
        steps.pack(side="left", fill="both", expand=True)
        ttk.Label(steps, text="Do this once per joint, with the motor OFF (press STOP). "
                              "Move the joint by hand and watch the position on the left.",
                  style="Note.TLabel").grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 6))

        ttk.Label(steps, text="A. Zero", style="Hdr.TLabel").grid(row=1, column=0, sticky="w")
        ttk.Label(steps, text="Put the joint at its reference pose (e.g. straight / aligned mark), then:").grid(
            row=2, column=0, columnspan=4, sticky="w")
        self.v_zero_save = tk.BooleanVar(value=False)
        ttk.Button(steps, text="Set zero HERE", command=self.do_zero).grid(row=3, column=0, sticky="w", pady=4)
        ttk.Checkbutton(steps, text="also save in motor flash (survives power-off)",
                        variable=self.v_zero_save).grid(row=3, column=1, columnspan=3, sticky="w")
        self.l_zero = ttk.Label(steps, text="", style="Note.TLabel")
        self.l_zero.grid(row=4, column=0, columnspan=4, sticky="w")

        ttk.Label(steps, text="B. Joint limits", style="Hdr.TLabel").grid(row=5, column=0, sticky="w", pady=(10, 0))
        ttk.Label(steps, text="Move slowly to the point where it would collide, then press the button. "
                              "The limit is stored 'inset' degrees before that point.").grid(
            row=6, column=0, columnspan=4, sticky="w")
        self.v_inset = self._entry(steps, "inset", 5.0, 7, unit="deg (safety gap)")
        ttk.Button(steps, text="◀ Set MIN here", command=lambda: self.teach("min")).grid(row=8, column=0, pady=4,
                                                                                    sticky="w")
        ttk.Button(steps, text="Set MAX here ▶", command=lambda: self.teach("max")).grid(row=8, column=1, pady=4,
                                                                                    sticky="w")
        self.v_lmin = self._entry(steps, "min", "-20", 9, unit="deg")
        self.v_lmax = self._entry(steps, "max", "20", 10, unit="deg")
        ttk.Button(steps, text="Apply typed limits", command=self.apply_limits).grid(row=11, column=0, pady=4,
                                                                                    sticky="w")
        self.v_free = tk.BooleanVar(value=False)
        ttk.Checkbutton(steps, text="Free shaft (nothing attached, no limits)", variable=self.v_free,
                        command=self.toggle_free).grid(row=11, column=1, columnspan=3, sticky="w")

        ttk.Label(steps, text="C. Common settings", style="Hdr.TLabel").grid(row=12, column=0, sticky="w",
                                                                         pady=(10, 0))
        self.v_cur = self._entry(steps, "current limit", 2.0, 13, unit="A (pos / vel modes, max 11)")
        self.v_tmax = self._entry(steps, "temp max", 70, 14, unit="°C (stop above)")
        ttk.Button(steps, text="Apply + save config", command=self.apply_common).grid(row=15, column=0, pady=4,
                                                                                     sticky="w")
        ttk.Label(steps, text="motor model").grid(row=16, column=0, sticky="w")
        self.v_model = tk.StringVar(value="RS05")
        cb = ttk.Combobox(steps, textvariable=self.v_model, values=list(MODELS), width=7, state="readonly")
        cb.grid(row=16, column=1, sticky="w")
        cb.bind("<<ComboboxSelected>>", lambda e: self.set_model())
        ttk.Label(steps, text="RS02 = 17 Nm / 44 rad/s scaling (wrong model = wrong torque/speed)",
                  style="Note.TLabel").grid(row=16, column=2, columnspan=3, sticky="w")
        help_ = ("Notes\n"
                 "• Zero moves the coordinate only, nothing moves physically.\n"
                 "• After zeroing, taught limits are shifted automatically\n"
                 "  so they stay at the same physical place.\n"
                 "• Limits + settings are saved per motor ID in\n"
                 "  rs05_gui_config.json and reloaded on connect.\n"
                 "• Motion is refused until limits are taught\n"
                 "  (or 'free shaft' is ticked).\n"
                 "• Position / velocity: target clamped, speed cut\n"
                 "  before the limit, STOP if >5° past it.\n"
                 "• MIT: damped virtual wall at the limit,\n"
                 "  STOP if >12° past it.")
        side = ttk.Frame(f)
        side.pack(side="left", anchor="n", padx=12)
        idf = ttk.LabelFrame(side, text=" D. Motor CAN ID ", padding=6)
        idf.pack(fill="x")
        ttk.Label(idf, text="New motors all ship as 0x7F (127). Give every motor its own ID:\n"
                            "connect ONE motor with that old ID on the bus, change it, repeat.\n"
                            "Suggested 4DOF: J1=1, J2=2, J3=3, J4=4 (unique even across buses).",
                  style="Note.TLabel", justify="left").grid(row=0, column=0, columnspan=3, sticky="w")
        ttk.Label(idf, text="connected motor").grid(row=1, column=0, sticky="w")
        self.l_curid = ttk.Label(idf, text="-", style="Hdr.TLabel")
        self.l_curid.grid(row=1, column=1, sticky="w")
        ttk.Label(idf, text="new ID (1-127)").grid(row=2, column=0, sticky="w")
        self.v_newid = tk.StringVar(value="1")
        ttk.Spinbox(idf, from_=1, to=127, textvariable=self.v_newid, width=6).grid(row=2, column=1, sticky="w")
        ttk.Button(idf, text="Change ID", command=self.change_id).grid(row=2, column=2, padx=4)
        ttk.Button(idf, text="Scan bus", command=self.scan_ids).grid(row=3, column=2, padx=4, pady=2)
        ttk.Button(idf, text="Deep scan (lost ID)…", command=self.deep_scan_ids).grid(
            row=3, column=0, columnspan=2, sticky="w", pady=2)
        self.v_deep_allbr = tk.BooleanVar(value=False)
        ttk.Checkbutton(idf, text="all bitrates (1M/500k/250k/125k)", variable=self.v_deep_allbr).grid(
            row=4, column=0, columnspan=3, sticky="w")
        ttk.Button(idf, text="Detect protocol (private / MIT / CANopen)…", command=self.detect_protocol).grid(
            row=5, column=0, columnspan=3, sticky="w", pady=2)
        self.l_idres = ttk.Label(idf, text="", wraplength=360, justify="left")
        self.l_idres.grid(row=6, column=0, columnspan=3, sticky="w")
        ttk.Label(side, text=help_, style="Note.TLabel", justify="left").pack(anchor="w", pady=(8, 0))

    # ------------------------------------------------------------------ 2 position
    def _build_pos_tab(self):
        f = self._tab(" 2  Position ")
        p = ttk.Frame(f)
        p.pack(side="left", anchor="n")
        self.v_pkind = tk.StringVar(value="pp")
        ttk.Radiobutton(p, text="PP  (motor plans a trapezoid: vmax + accel)", variable=self.v_pkind,
                        value="pp").grid(row=0, column=0, columnspan=3, sticky="w")
        ttk.Radiobutton(p, text="CSP (follows target at speed limit)", variable=self.v_pkind,
                        value="csp").grid(row=1, column=0, columnspan=3, sticky="w")
        self.v_pvmax = self._entry(p, "max speed", 2.0, 2, unit="rad/s")
        self.v_pacc = self._entry(p, "accel (PP)", 5.0, 3, unit="rad/s²")
        ttk.Button(p, text="Apply params", command=self.apply_pos_params).grid(row=4, column=0, pady=4, sticky="w")
        ttk.Button(p, text="ENABLE (hold here)", command=self.pos_enable).grid(row=5, column=0, pady=(10, 2),
                                                                             sticky="we")
        ttk.Button(p, text="Disable", command=self.stop_motor).grid(row=5, column=1, pady=(10, 2), sticky="we")

        t = ttk.Frame(f)
        t.pack(side="left", fill="both", expand=True, padx=20)
        ttk.Label(t, text="Target (deg) - slider is limited to the joint limits; released = sent",
                  style="Hdr.TLabel").pack(anchor="w")
        self.s_ptarget = tk.Scale(t, from_=-20, to=20, orient="horizontal", resolution=0.5, length=600)
        self.s_ptarget.pack(fill="x")
        self.s_ptarget.bind("<ButtonRelease-1>", lambda e: self.pos_go(self.s_ptarget.get()))
        row = ttk.Frame(t)
        row.pack(anchor="w", pady=4)
        for d in (-10, -5, -1, 1, 5, 10):
            ttk.Button(row, text=f"{d:+d}°", width=5, command=lambda d=d: self.pos_jog(d)).pack(side="left", padx=1)
        ttk.Button(row, text="Go 0°", command=lambda: self.pos_go(0.0)).pack(side="left", padx=(10, 1))
        ttk.Button(row, text="Go mid", command=self.pos_go_mid).pack(side="left", padx=1)
        row2 = ttk.Frame(t)
        row2.pack(anchor="w", pady=4)
        ttk.Label(row2, text="exact target").pack(side="left")
        self.v_ptarget = tk.StringVar(value="0")
        ttk.Entry(row2, textvariable=self.v_ptarget, width=8).pack(side="left", padx=4)
        ttk.Button(row2, text="Go", command=lambda: self.pos_go(fnum(self.v_ptarget))).pack(side="left")
        self.l_pinfo = ttk.Label(t, text="", style="Big.TLabel")
        self.l_pinfo.pack(anchor="w", pady=6)

    # ------------------------------------------------------------------ 3 velocity
    def _build_vel_tab(self):
        f = self._tab(" 3  Velocity ")
        p = ttk.Frame(f)
        p.pack(side="left", anchor="n")
        self.v_vacc = self._entry(p, "accel", 5.0, 0, unit="rad/s²")
        self.v_vmax = self._entry(p, "slider range ±", 10.0, 1, unit="rad/s")
        self.v_vjog = self._entry(p, "hold-jog speed", 1.0, 2, unit="rad/s")
        ttk.Button(p, text="Apply params", command=self.apply_vel_params).grid(row=3, column=0, pady=4, sticky="w")
        ttk.Button(p, text="ENABLE (0 rad/s)", command=self.vel_enable).grid(row=4, column=0, pady=(10, 2),
                                                                           sticky="we")
        ttk.Button(p, text="Disable", command=self.stop_motor).grid(row=4, column=1, pady=(10, 2), sticky="we")

        t = ttk.Frame(f)
        t.pack(side="left", fill="both", expand=True, padx=20)
        ttk.Label(t, text="Speed (rad/s) - released = sent. Speed is cut to 0 automatically before a limit.",
                  style="Hdr.TLabel").pack(anchor="w")
        self.s_vel = tk.Scale(t, from_=-10, to=10, orient="horizontal", resolution=0.05, length=600)
        self.s_vel.pack(fill="x")
        self.s_vel.bind("<ButtonRelease-1>", lambda e: self.vel_set(self.s_vel.get()))
        row = ttk.Frame(t)
        row.pack(anchor="w", pady=4)
        for v in (-2, -1, -0.5, 0, 0.5, 1, 2):
            ttk.Button(row, text=f"{v:+g}" if v else "0", width=5,
                       command=lambda v=v: self.vel_set(v, move_slider=True)).pack(side="left", padx=1)
        row2 = ttk.Frame(t)
        row2.pack(anchor="w", pady=8)
        ttk.Label(row2, text="Hold to jog:").pack(side="left")
        for sign, txt in ((-1, "◀◀  hold"), (1, "hold  ▶▶")):
            b = ttk.Button(row2, text=txt, width=12)
            b.pack(side="left", padx=4)
            b.bind("<ButtonPress-1>", lambda e, s=sign: self.vel_set(s * fnum(self.v_vjog, 1.0)))
            b.bind("<ButtonRelease-1>", lambda e: self.vel_set(0.0))
        self.l_vinfo = ttk.Label(t, text="", style="Big.TLabel")
        self.l_vinfo.pack(anchor="w", pady=6)

    # ------------------------------------------------------------------ 4 MIT
    def _build_mit_tab(self):
        f = self._tab(" 4  MIT / Impedance ")
        left = ttk.Frame(f)
        left.pack(side="left", anchor="n")
        ttk.Label(left, text="τ = Kp·(θd − θ) + Kd·(ωd − ω) + τff", style="Big.TLabel").pack(anchor="w")
        ttk.Label(left, text="Impedance control: Kp = stiffness (Nm/rad), Kd = damping (Nm·s/rad).\n"
                             "Start, then push the joint by hand to feel it. Sliders act live.",
                  style="Note.TLabel").pack(anchor="w", pady=(0, 6))
        pre = ttk.Frame(left)
        pre.pack(anchor="w")
        for name, kp, kd in MIT_PRESETS:
            ttk.Button(pre, text=name, command=lambda kp=kp, kd=kd: self.mit_preset(kp, kd)).pack(side="left",
                                                                                              padx=1)
        btn = ttk.Frame(left)
        btn.pack(anchor="w", pady=6)
        ttk.Button(btn, text="START MIT (hold here)", command=self.mit_start).pack(side="left")
        ttk.Button(btn, text="Target = here", command=self.mit_here).pack(side="left", padx=4)
        ttk.Button(btn, text="Stop", command=self.stop_motor).pack(side="left")
        self.l_minfo = ttk.Label(left, text="", style="Big.TLabel", justify="left")
        self.l_minfo.pack(anchor="w", pady=6)

        s = ttk.Frame(f)
        s.pack(side="left", fill="both", expand=True, padx=16)
        mk = lambda lab, lo, hi, res, init: self._mscale(s, lab, lo, hi, res, init)  # noqa: E731
        self.s_kp = mk("Kp  stiffness  Nm/rad", 0, 60, 0.5, 10)
        self.s_kd = mk("Kd  damping  Nm·s/rad", 0, KD_MAX, 0.01, 0.5)
        self.s_tff = mk("τff  feed-forward  Nm", -1, 1, 0.01, 0)
        self.s_mtarget = mk("θd  target  deg", -20, 20, 0.5, 0)
        self.s_ramp = mk("target ramp  rad/s", 0.1, 10, 0.1, 2)

    def _mscale(self, parent, label, lo, hi, res, init):
        sc = tk.Scale(parent, label=label, from_=lo, to=hi, orient="horizontal", resolution=res, length=520,
                      command=lambda _v: self.mit_push())
        sc.set(init)
        sc.pack(fill="x")
        return sc

    # ------------------------------------------------------------------ 5 friction
    def _build_friction_tab(self):
        f = self._tab(" 5  Friction ")
        col = ttk.Frame(f)
        col.pack(side="left", anchor="n")
        drv = ttk.Frame(col)
        drv.pack(anchor="w", pady=(0, 4))
        ttk.Label(drv, text="Sweep drive", style="Hdr.TLabel").grid(row=0, column=0, sticky="w")
        self.v_fdrive = tk.StringVar(value="vel")
        for i, (key, name) in enumerate(FRICTION_DRIVES.items()):
            ttk.Radiobutton(drv, text=name, variable=self.v_fdrive, value=key).grid(row=0, column=1 + i, sticky="w",
                                                                                   padx=2)
        self.v_fkp = self._entry(drv, "MIT Kp", 2.0, 1, width=6)
        self.v_fkd = self._entry(drv, "MIT Kd", 1.0, 1, col=3, width=6)
        ttk.Label(drv, style="Note.TLabel", justify="left",
                  text="velocity loop = reference.  CSP = position loop, limit_spd = v.\n"
                       "MIT: θd ramps at v, ωd = v. Keep Kp small (≈2): θd goes out\n"
                       "at 100 Hz as a staircase -> torque ripple. Kd does the tracking.").grid(
            row=2, column=0, columnspan=6, sticky="w")
        p = ttk.Frame(col)
        p.pack(anchor="w")
        ttk.Label(p, text="Constant-speed passes between the limits → τ = Fc·sign(ω) + B·ω.\n"
                          "Then a slow torque ramp → breakaway (static) friction.\n"
                          "Remove gravity load (joint horizontal or link off):\n"
                          "everything on the shaft is measured.", style="Note.TLabel").grid(
            row=0, column=0, columnspan=3, sticky="w", pady=(0, 6))
        self.v_fspeeds = self._entry(p, "speeds", "0.3 0.6 1 1.5 2 3", 1, width=18, unit="rad/s")
        self.v_fpasses = self._entry(p, "passes / dir", 2, 2)
        self.v_facc = self._entry(p, "accel", 10.0, 3, unit="rad/s²")
        self.v_fedge = self._entry(p, "ignore near limits", 5.0, 4, unit="deg")
        self.v_fbreak = tk.BooleanVar(value=True)
        ttk.Checkbutton(p, text="breakaway test", variable=self.v_fbreak).grid(row=5, column=0, sticky="w")
        self.v_ftmax = self._entry(p, "breakaway τ max", 0.8, 6, unit="Nm")
        self.v_frate = self._entry(p, "torque ramp", 0.1, 7, unit="Nm/s")
        self.l_frange = ttk.Label(p, text="", style="Note.TLabel")
        self.l_frange.grid(row=8, column=0, columnspan=3, sticky="w")
        b = ttk.Frame(p)
        b.grid(row=9, column=0, columnspan=3, sticky="w", pady=6)
        ttk.Button(b, text="RUN", command=self.friction_run).pack(side="left")
        ttk.Button(b, text="Abort", command=self.friction_abort).pack(side="left", padx=4)
        ttk.Button(b, text="Save results", command=self.friction_save).pack(side="left")
        self.pb_f = ttk.Progressbar(p, length=300)
        self.pb_f.grid(row=10, column=0, columnspan=3, sticky="w")
        self.l_fstat = ttk.Label(p, text="", wraplength=380)
        self.l_fstat.grid(row=11, column=0, columnspan=3, sticky="w")

        self.t_fres = tk.Text(f, width=46, height=13, font=("Consolas", 9))
        self.t_fres.pack(side="left", anchor="n", padx=8)
        self.ffig = Figure(figsize=(3.6, 2.8), dpi=90)
        self.fax = self.ffig.add_subplot(111)
        self.fax.set_xlabel("ω [rad/s]")
        self.fax.set_ylabel("τ [Nm]")
        self.ffig.tight_layout()
        self.fcanvas = FigureCanvasTkAgg(self.ffig, master=f)
        self.fcanvas.get_tk_widget().pack(side="left", fill="both", expand=True)

    # ------------------------------------------------------------------ 6 tuning
    def _build_tune_tab(self):
        f = self._tab(" 6  Tuning (step) ")
        p = ttk.Frame(f)
        p.pack(side="left", anchor="n")
        ttk.Label(p, text="Step response: start → start+step → start …  (stays inside the limits)",
                  style="Note.TLabel").grid(row=0, column=0, columnspan=4, sticky="w")
        self.v_smode = tk.StringVar(value="mit")
        mf = ttk.Frame(p)
        mf.grid(row=1, column=0, columnspan=4, sticky="w", pady=2)
        ttk.Label(mf, text="Control", style="Hdr.TLabel").pack(side="left")
        for key, name in STEP_MODES.items():
            ttk.Radiobutton(mf, text=name, variable=self.v_smode, value=key).pack(side="left", padx=3)
        self.v_sstep = self._entry(p, "step", 10.0, 2, width=6, unit="deg")
        self.v_sreps = self._entry(p, "repeats", 3, 3, width=6, unit="up+down pairs")
        self.v_sdwell = self._entry(p, "dwell", 1.5, 4, width=6, unit="s per step")
        self.v_sband = self._entry(p, "settle band", 2.0, 5, width=6, unit="% of step (≥0.2°)")
        ttk.Label(p, text="MIT", style="Hdr.TLabel").grid(row=6, column=0, sticky="w", pady=(6, 0))
        self.v_skp = self._entry(p, "Kp", 10.0, 7, width=6, unit="Nm/rad")
        self.v_skd = self._entry(p, "Kd", 0.5, 8, width=6, unit="Nm·s/rad")
        self.v_sramp = self._entry(p, "θd ramp", 0.0, 9, width=6, unit="rad/s (0 = true step)")
        ttk.Label(p, text="Position PP / CSP", style="Hdr.TLabel").grid(row=10, column=0, sticky="w", pady=(6, 0))
        self.v_svmax = self._entry(p, "max speed", 3.0, 11, width=6, unit="rad/s")
        self.v_sacc = self._entry(p, "accel (PP)", 20.0, 12, width=6, unit="rad/s²")
        self.v_scur = self._entry(p, "current limit", 2.0, 13, width=6, unit="A")
        b = ttk.Frame(p)
        b.grid(row=14, column=0, columnspan=4, sticky="w", pady=6)
        ttk.Button(b, text="RUN", command=self.step_run).pack(side="left")
        ttk.Button(b, text="Abort", command=self.stop_motor).pack(side="left", padx=4)
        ttk.Button(b, text="Save results", command=self.step_save).pack(side="left")
        self.pb_s = ttk.Progressbar(p, length=260)
        self.pb_s.grid(row=15, column=0, columnspan=4, sticky="w")
        self.l_sstat = ttk.Label(p, text="", wraplength=330)
        self.l_sstat.grid(row=16, column=0, columnspan=4, sticky="w")

        g = ttk.LabelFrame(f, text=" Motor loop gains (PP / CSP / velocity - MIT ignores them) ", padding=6)
        g.pack(side="left", anchor="n", padx=8)
        self.v_gain = {}
        for i, k in enumerate(LOOP_GAINS):
            self.v_gain[k] = self._entry(g, k, "", i, width=10)
        gb = ttk.Frame(g)
        gb.grid(row=len(LOOP_GAINS), column=0, columnspan=3, sticky="w", pady=4)
        ttk.Button(gb, text="Read", command=self.gains_read).pack(side="left")
        ttk.Button(gb, text="Write (RAM)", command=self.gains_write).pack(side="left", padx=3)
        ttk.Button(gb, text="Restore read", command=self.gains_restore).pack(side="left")
        ttk.Button(g, text="Save ALL params to motor flash…", command=self.gains_flash).grid(
            row=len(LOOP_GAINS) + 1, column=0, columnspan=3, sticky="w")
        ttk.Label(g, style="Note.TLabel", justify="left", wraplength=230,
                  text="Read first. Change one gain at a time in small steps (±20-30 %) and compare step "
                       "responses. RAM values are lost at power-off. Pos hold buzzing / heating: try lower "
                       "spd_kp or spd_ki. Slow, soft position response: raise loc_kp.").grid(
            row=len(LOOP_GAINS) + 2, column=0, columnspan=3, sticky="w", pady=(6, 0))

        right = ttk.Frame(f)
        right.pack(side="left", fill="both", expand=True)
        self.t_sres = tk.Text(right, width=44, height=7, font=("Consolas", 9))
        self.t_sres.pack(fill="x")
        self.sfig = Figure(figsize=(3.8, 2.8), dpi=90)
        self.sax_y, self.sax_t = self.sfig.subplots(2, 1, sharex=True)
        self.sfig.tight_layout()
        self.scanvas = FigureCanvasTkAgg(self.sfig, master=right)
        self.scanvas.get_tk_widget().pack(fill="both", expand=True)

    # ================================================================== live plot
    def _build_plot(self, parent):
        self.fig = Figure(figsize=(9, 4.5), dpi=90)
        self.ax_p, self.ax_v, self.ax_t = self.fig.subplots(3, 1, sharex=True)
        self.ln_p, = self.ax_p.plot([], [], lw=1.4, label="position")
        self.ln_ps, = self.ax_p.plot([], [], "--", lw=1.0, label="setpoint")
        self.ln_lo = self.ax_p.axhline(0, color="#c62828", lw=0.8, ls=":")
        self.ln_hi = self.ax_p.axhline(0, color="#c62828", lw=0.8, ls=":")
        self.ax_p.set_ylabel("deg")
        self.ax_p.legend(loc="upper left", fontsize=7)
        self.ln_vraw, = self.ax_v.plot([], [], lw=0.8, color="#bbb", label="raw (motor estimate)")
        self.ln_v, = self.ax_v.plot([], [], lw=1.4, color="#1f77b4", label=f"velocity (median {VEL_FILTER_N})")
        self.ln_vs, = self.ax_v.plot([], [], "--", lw=1.0, label="setpoint")
        self.ax_v.set_ylabel("rad/s")
        self.ax_v.legend(loc="upper left", fontsize=7)
        self.ln_t, = self.ax_t.plot([], [], lw=1.2, label="torque Nm")
        self.ln_i, = self.ax_t.plot([], [], lw=1.0, label="iq A")
        self.ax_t.set_ylabel("Nm / A")
        self.ax_t.set_xlabel("time [s]")
        self.ax_t.legend(loc="upper left", fontsize=7)
        for ax in (self.ax_p, self.ax_v, self.ax_t):
            ax.grid(alpha=0.3)
        self.fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, pady=(6, 0))

    # ================================================================== connection
    def toggle_connect(self):
        if self.c:
            self.disconnect()
        else:
            self.connect()

    def _open(self):
        iface = self.v_iface.get()
        if iface == "sim":
            if self.sim is None:
                from rs05_sim import FakeRS05
                self.sim = FakeRS05(SIM_CHANNEL, int(self.v_id.get(), 0), int(self.v_host.get(), 0))
            return can.Bus(interface="virtual", channel=SIM_CHANNEL)
        if iface == "stm-sim":
            from rs05_stm import StmBus
            from rs05_stm_emu import StmEmulator
            if self.stm_emu is None:
                self.stm_emu = StmEmulator()
                self.stm_emu.add_motor(0, int(self.v_id.get(), 0))     # one simulated motor on CAN1
            return StmBus("stm-sim", serial_port=self.stm_emu.connect())
        if iface == "stm" and int(self.v_bitrate.get()) != 1_000_000:
            self.v_bitrate.set("1000000")                               # STM firmware: 1 Mbit/s fixed
        return open_bus(iface, self.v_port.get(), int(self.v_bitrate.get()))

    def connect(self):
        try:
            mid, host = int(self.v_id.get(), 0), int(self.v_host.get(), 0)
            self.bus = self._open()
            self.m = RS05(self.bus, mid, host)
            self.m.stop()
            self.m.active_report(True)
            self.m.start_polling(20)
            if self.m.wait_feedback(1.0) is None:
                self.m.stop()
                if self.m.wait_feedback(0.5) is None:
                    raise RuntimeError(f"No feedback from motor 0x{mid:02X}.\nCheck 48 V, CAN H/L, 120 Ω "
                                       f"termination, bitrate, motor ID (Scan IDs).")
        except Exception as e:  # noqa: BLE001
            self._teardown()
            messagebox.showerror("Connect failed", str(e))
            return
        cfg = load_config(mid)
        self.c = Controller(self.m, cfg)
        self._load_cfg_into_ui()
        self.b_conn.config(text="Disconnect")
        self.l_conn.config(text=f"connected: {self.v_iface.get()} {self.v_port.get()}  motor 0x{mid:02X}")
        self.l_curid.config(text=f"0x{mid:02X} ({mid})")
        self.c.log(f"connected to motor 0x{mid:02X}")

    def _teardown(self):
        if self.ft and not self.ft.done:
            self.ft.abort()
        if self.c:
            self.c.close()
        if self.m:
            try:
                self.m.active_report(False)
            except can.CanError:
                pass
            self.m.close()
        if self.bus:
            self.bus.shutdown()
        self.stop_record()
        self.c = self.m = self.bus = None

    def disconnect(self):
        if self.c:
            save_config(self.m.id, self.c.cfg)
        self._teardown()
        self.b_conn.config(text="Connect")
        self.l_conn.config(text="disconnected")
        self.l_curid.config(text="-")

    def scan_ids(self):
        tmp_bus = None
        try:
            if self.m:
                if self.c and self.c.mode != "idle":
                    raise SafetyTrip("Stop the motor before scanning.")
                ids = self.m.scan()
            else:
                tmp_bus = self._open()
                m = RS05(tmp_bus, 0, int(self.v_host.get(), 0))
                ids = m.scan()
                m.close()
            self.cb_id["values"] = [hex(i) for i in ids]
            txt = ", ".join(f"0x{i:02X} ({i})" for i in ids) or "none"
            self.l_idres.config(text=f"on the bus: {txt}")
            if ids and not self.m and int(self.v_id.get(), 0) not in ids:
                self.v_id.set(hex(ids[0]))
            messagebox.showinfo("Scan", f"Motors answering: {txt}\n\nPick one in 'Motor ID' and Connect."
                                if not self.m else f"Motors answering: {txt}")
        except Exception as e:  # noqa: BLE001
            messagebox.showerror("Scan failed", str(e))
        finally:
            if tmp_bus:
                tmp_bus.shutdown()

    def deep_scan_ids(self):
        """Lost / undocumented ID: IDs 0..255, get-ID + stop probe, passive listen, optional all bitrates."""
        if getattr(self, "_deep_running", False):
            return
        if self.m:
            if not messagebox.askyesno("Deep scan", "Deep scan needs the CANable for itself.\nDisconnect now?"):
                return
            self.disconnect()
        allbr = self.v_deep_allbr.get() and self.v_iface.get() not in ONE_BITRATE
        if not messagebox.askyesno(
                "Deep scan",
                "Sends 'get ID' and 'stop' to every ID 0..255"
                + (" at 1M, 500k, 250k and 125k bit/s" if allbr else "") +
                ".\n'stop' disables any running motor - nothing on the bus should be moving.\n\nContinue?"):
            return
        iface, port, host = self.v_iface.get(), self.v_port.get(), int(self.v_host.get(), 0)
        bitrate = int(self.v_bitrate.get())
        self._deep_running = True
        self.l_idres.config(text="deep scan running…")

        def prog(f, br=None):
            self.after(0, lambda: self.l_idres.config(
                text=f"deep scan {100 * f:.0f} %" + (f"  ({br} bit/s)" if br else "")))

        def work():
            try:
                if allbr:
                    results = deep_scan_bitrates(iface, port, host, progress=prog)
                else:
                    bus = self._open()
                    try:
                        results = [deep_scan(bus, host, bitrate=None if iface in ("sim", "stm-sim") else bitrate,
                                             progress=prog)]
                    finally:
                        bus.shutdown()
                self.after(0, lambda: self._deep_done(results))
            except Exception as e:  # noqa: BLE001
                self.after(0, lambda e=e: self._deep_done([], str(e)))

        threading.Thread(target=work, daemon=True).start()

    def _deep_done(self, results, err: str = ""):
        self._deep_running = False
        if err:
            self.l_idres.config(text=f"deep scan failed: {err}")
            messagebox.showerror("Deep scan failed", err)
            return
        found = [(r.bitrate, cid) for r in results for cid in sorted(r.motors)]
        txt = "\n".join(r.text() for r in results)
        self.l_idres.config(text=txt)
        if not found:
            messagebox.showwarning(
                "Deep scan", txt + "\n\nNo RS05 answered. Check 48 V, CAN H/L, 120 Ω termination; tick "
                "'all bitrates'.\nIf other traffic with 11-bit IDs is listed, the motor was switched to "
                "MIT/CANopen protocol - switch it back to the private protocol with the RobStride software.")
            return
        self.cb_id["values"] = sorted({hex(cid) for _, cid in found})
        br, cid = found[0]
        self.v_id.set(hex(cid))
        if br:
            self.v_bitrate.set(str(br))
        messagebox.showinfo(
            "Deep scan", txt + f"\n\nMotor ID set to 0x{cid:02X}" + (f" at {br} bit/s" if br else "") +
            ".\nConnect, then use 'Change ID' to give it a documented ID (1..127).")

    def detect_protocol(self):
        """Is the motor in PRIVATE, MIT or CANopen mode? Offers to switch MIT / CANopen motors back."""
        from rs05_protocol import CANOPEN, MIT, probe, probe_bitrates
        if getattr(self, "_deep_running", False):
            return
        if self.m:
            if not messagebox.askyesno("Detect protocol", "Needs the CANable for itself.\nDisconnect now?"):
                return
            self.disconnect()
        allbr = self.v_deep_allbr.get() and self.v_iface.get() not in ONE_BITRATE
        if not messagebox.askyesno(
                "Detect protocol",
                "Probes private (get ID), MIT ('stop' = disable) and CANopen (SDO read)"
                + (" at 1M/500k/250k/125k" if allbr else "") +
                ".\nNothing is enabled or moved, but a running motor is stopped.\n\nContinue?"):
            return
        iface, port, host = self.v_iface.get(), self.v_port.get(), int(self.v_host.get(), 0)
        bitrate = int(self.v_bitrate.get())
        self._deep_running = True

        def prog(name, f):
            self.after(0, lambda: self.l_idres.config(text=f"probing {name} {100 * f:.0f} %"))

        def work():
            try:
                if allbr:
                    reps = probe_bitrates(iface, port, host, progress=prog)
                else:
                    bus = self._open()
                    try:
                        reps = [probe(bus, host, bitrate=None if iface in ("sim", "stm-sim") else bitrate, progress=prog)]
                    finally:
                        bus.shutdown()
                self.after(0, lambda: self._detect_done(reps, MIT, CANOPEN))
            except Exception as e:  # noqa: BLE001
                self.after(0, lambda e=e: self._detect_done([], MIT, CANOPEN, str(e)))

        threading.Thread(target=work, daemon=True).start()

    def _detect_done(self, reps, MIT, CANOPEN, err=""):
        from rs05_protocol import switch_to_private
        self._deep_running = False
        if err:
            self.l_idres.config(text=f"probe failed: {err}")
            messagebox.showerror("Detect protocol", err)
            return
        txt = "\n".join(r.text() for r in reps)
        found = [(r, f) for r in reps for f in r.findings]
        self.l_idres.config(text=txt)
        advice = "\n".join(r.advice() for r in reps if r.findings) or reps[0].advice()
        messagebox.showinfo("Detect protocol", txt + "\n\n" + advice)
        for r, f in found:
            if f.protocol not in (MIT, CANOPEN):
                continue
            who = f"motor 0x{f.motor_id:02X} (MIT)" if f.protocol == MIT else f"CANopen node {f.motor_id}"
            if not messagebox.askyesno(
                    "Switch to private protocol",
                    f"{who} is not in the private protocol.\nSend 'switch to PRIVATE' now?\n"
                    "(takes effect after a motor power cycle"
                    + ("; CANopen frame is from the manual and marked VERIFY" if f.protocol == CANOPEN else "")
                    + ")"):
                continue
            try:
                if r.bitrate and r.bitrate != int(self.v_bitrate.get()):
                    self.v_bitrate.set(str(r.bitrate))
                bus = self._open()
                try:
                    ok, msg = switch_to_private(bus, f.protocol, f.motor_id)
                finally:
                    bus.shutdown()
                if self.sim and ok and hasattr(self.sim, "power_cycle"):
                    self.sim.power_cycle()
                (messagebox.showinfo if ok else messagebox.showwarning)("Switch to private", msg)
            except Exception as e:  # noqa: BLE001
                messagebox.showerror("Switch to private", str(e))

    def change_id(self):
        def go():
            if self.c.mode != "idle":
                raise SafetyTrip("Press STOP first.")
            old = self.m.id
            try:
                new = int(self.v_newid.get(), 0)
            except ValueError:
                raise ValueError("new ID must be a number 1..127")
            if not 1 <= new <= 127 or new == self.m.host:
                raise ValueError(f"new ID must be 1..127 and not the host ID 0x{self.m.host:02X}")
            if new == old:
                return
            if self.m.ping(new):
                raise SafetyTrip(f"ID 0x{new:02X} ({new}) is already used by another motor on this bus.")
            if not messagebox.askyesno(
                    "Change CAN ID", f"Change motor 0x{old:02X} ({old}) → 0x{new:02X} ({new})?\n\n"
                                     f"Only ONE motor with ID {old} may be on the bus right now\n"
                                     f"(all motors with that ID would change)."):
                return
            ok = self.m.set_can_id(new)
            if not ok:
                self.l_idres.config(text=f"FAILED: no answer on ID {new}. Motor still on {old}? Scan the bus.")
                self.c.log(f"CAN ID change {old} -> {new} FAILED")
                return
            cfg = self.c.cfg
            self.disconnect()
            move_config(old, new, cfg)
            self.v_id.set(hex(new))
            self.connect()
            self.l_idres.config(text=f"OK: motor is now 0x{new:02X} ({new}). Settings moved with it.\n"
                                     f"Power-cycle it and Scan to confirm the ID is kept.")
            if self.c:
                self.c.log(f"CAN ID changed {old} -> {new}")
        self.call(go)

    def on_close(self):
        try:
            self.disconnect()
        finally:
            if self.sim:
                self.sim.close()
            if self.stm_emu:
                self.stm_emu.close()
            self.destroy()

    # ================================================================== helpers
    def call(self, fn):
        if not self.c:
            messagebox.showwarning("Not connected", "Connect to the motor first.")
            return None
        try:
            return fn()
        except SafetyTrip as e:
            messagebox.showwarning("Refused", str(e))
        except (TimeoutError, can.CanError, ValueError) as e:
            messagebox.showerror("Error", str(e))
        return None

    def stop_motor(self):
        if self.ft and not self.ft.done:
            self.ft.abort()
        if self.st and not self.st.done:
            self.st.abort()
        if self.c:
            self.c.stop()

    def _load_cfg_into_ui(self):
        c = self.c.cfg
        self.v_lmin.set(f"{c.lim_min_deg:.1f}")
        self.v_lmax.set(f"{c.lim_max_deg:.1f}")
        self.v_free.set(c.free_shaft)
        self.v_model.set(c.model)
        self.v_cur.set(c.cur_limit)
        self.v_tmax.set(c.temp_max)
        self.v_pvmax.set(c.pos_vmax)
        self.v_pacc.set(c.pos_acc)
        self.v_vacc.set(c.vel_acc)
        self.v_vmax.set(c.vel_max)
        self.s_vel.config(from_=-c.vel_max, to=c.vel_max)
        self.s_kp.config(to=c.mit_kp_max)
        self.s_tff.config(from_=-c.mit_tff_max, to=c.mit_tff_max)
        self.s_kp.set(c.mit_kp)
        self.s_kd.set(c.mit_kd)
        self.s_ramp.set(c.mit_ramp)
        self._last_lim = None

    def save_cfg(self):
        if self.c:
            save_config(self.m.id, self.c.cfg)

    # ================================================================== setup actions
    def do_zero(self):
        def go():
            pos = math.degrees(self.c.fb.pos)
            if not messagebox.askyesno("Set zero", f"Joint is at {pos:+.2f}°.\nMake THIS pose 0°?"
                                       + ("\n(also saved to motor flash)" if self.v_zero_save.get() else "")):
                return
            old, new, ok = self.c.zero_here(self.v_zero_save.get())
            self.l_zero.config(text=f"{'OK' if ok else 'FAILED'}: {math.degrees(old):+.2f}° → "
                                    f"{math.degrees(new):+.2f}°" + ("  (limits shifted)" if
                                                                     self.c.cfg.limits_taught else ""))
            self._load_cfg_into_ui()
            self.save_cfg()
        self.call(go)

    def teach(self, which):
        def go():
            if self.c.mode != "idle":
                raise SafetyTrip("Press STOP first; teach limits with the motor off.")
            inset = abs(fnum(self.v_inset, 5.0))
            deg = math.degrees(self.c.fb.pos)
            if which == "min":
                self.v_lmin.set(f"{deg + inset:.1f}")
            else:
                self.v_lmax.set(f"{deg - inset:.1f}")
            self.c.log(f"{which} end at {deg:+.1f}° → limit {deg + inset if which == 'min' else deg - inset:+.1f}°")
            lo, hi = fnum(self.v_lmin), fnum(self.v_lmax)
            if hi - lo >= 2:
                self.c.set_limits(lo, hi)
                self.save_cfg()
            else:
                self.c.log("now teach the other end")
        self.call(go)

    def apply_limits(self):
        def go():
            self.c.set_limits(fnum(self.v_lmin), fnum(self.v_lmax))
            self.save_cfg()
        self.call(go)

    def toggle_free(self):
        def go():
            if self.v_free.get() and not messagebox.askyesno(
                    "Free shaft", "Disable joint limits? Only if NOTHING is attached to the shaft."):
                self.v_free.set(False)
                return
            self.c.cfg.free_shaft = self.v_free.get()
            self.c.log(f"free shaft {'ON' if self.c.cfg.free_shaft else 'off'}")
            self.save_cfg()
        self.call(go)

    def set_model(self):
        def go():
            if self.c.mode != "idle":
                self.v_model.set(self.c.cfg.model)
                raise SafetyTrip("STOP the motor before changing the model.")
            self.c.cfg.model = self.v_model.get()
            self.m.set_model(self.c.cfg.model)
            self.c.log(f"motor model {self.c.cfg.model}: +-{self.m.t_max} Nm, +-{self.m.v_max} rad/s")
            self.save_cfg()
        self.call(go)

    def apply_common(self):
        def go():
            self.c.cfg.cur_limit = max(0.1, min(11.0, fnum(self.v_cur, 2.0)))
            self.c.cfg.temp_max = max(30.0, min(90.0, fnum(self.v_tmax, 70)))
            self.c.update_pos_params()
            self.c.update_vel_params()
            self.c.log(f"current limit {self.c.cfg.cur_limit} A, temp max {self.c.cfg.temp_max} C")
            self.save_cfg()
        self.call(go)

    # ================================================================== position actions
    def apply_pos_params(self):
        def go():
            self.c.cfg.pos_vmax = max(0.05, min(50.0, fnum(self.v_pvmax, 2.0)))
            self.c.cfg.pos_acc = max(0.1, min(200.0, fnum(self.v_pacc, 5.0)))
            self.c.update_pos_params()
            self.save_cfg()
        self.call(go)

    def pos_enable(self):
        self.apply_pos_params()
        self.call(lambda: self.c.start_pos(self.v_pkind.get()))
        if self.c and self.c.mode == "pos":
            self.s_ptarget.set(math.degrees(self.c.pos_target))

    def pos_go(self, deg):
        def go():
            if self.c.mode != "pos":
                raise SafetyTrip("Press ENABLE first.")
            self.c.set_pos_target(math.radians(deg))
            self.s_ptarget.set(math.degrees(self.c.pos_target))
            self.v_ptarget.set(f"{math.degrees(self.c.pos_target):.1f}")
        self.call(go)

    def pos_jog(self, d):
        if self.c:
            self.pos_go(math.degrees(self.c.pos_target) + d)

    def pos_go_mid(self):
        if self.c:
            lo, hi = self.c.limits_rad()
            self.pos_go(math.degrees((lo + hi) / 2))

    # ================================================================== velocity actions
    def apply_vel_params(self):
        def go():
            self.c.cfg.vel_acc = max(0.1, min(200.0, fnum(self.v_vacc, 5.0)))
            self.c.cfg.vel_max = max(0.1, min(50.0, fnum(self.v_vmax, 10.0)))
            self.s_vel.config(from_=-self.c.cfg.vel_max, to=self.c.cfg.vel_max)
            self.c.update_vel_params()
            self.save_cfg()
        self.call(go)

    def vel_enable(self):
        self.apply_vel_params()
        self.call(self.c.start_vel if self.c else lambda: None)
        self.s_vel.set(0)

    def vel_set(self, v, move_slider=False):
        def go():
            if self.c.mode != "vel":
                raise SafetyTrip("Press ENABLE first.")
            self.c.set_vel(v)
            if move_slider:
                self.s_vel.set(v)
        self.call(go)

    # ================================================================== MIT actions
    def mit_preset(self, kp, kd):
        self.s_kp.set(kp)
        self.s_kd.set(kd)
        self.mit_push()

    def mit_push(self):
        if not self.c:
            return
        self.c.set_mit(target=math.radians(self.s_mtarget.get()), kp=self.s_kp.get(), kd=self.s_kd.get(),
                       tff=self.s_tff.get(), ramp=self.s_ramp.get())
        self.c.cfg.mit_kp, self.c.cfg.mit_kd, self.c.cfg.mit_ramp = self.s_kp.get(), self.s_kd.get(), \
            self.s_ramp.get()

    def mit_start(self):
        def go():
            self.c.set_mit(kp=self.s_kp.get(), kd=self.s_kd.get(), tff=self.s_tff.get(), ramp=self.s_ramp.get())
            self.c.start_mit()
            self.s_mtarget.set(math.degrees(self.c.mit_target))
            self.save_cfg()
        self.call(go)

    def mit_here(self):
        if self.c and self.c.fb:
            self.s_mtarget.set(math.degrees(self.c.clamp_to_limits(self.c.fb.pos)))
            self.mit_push()

    # ================================================================== friction actions
    def _fparams(self):
        speeds = [abs(float(x)) for x in self.v_fspeeds.get().replace(",", " ").split()]
        if not speeds:
            raise ValueError("enter at least one speed")
        return FrictionParams(drive=self.v_fdrive.get(), mit_kp=max(0.0, fnum(self.v_fkp, 2.0)),
                              mit_kd=max(0.0, fnum(self.v_fkd, 1.0)),
                              speeds=speeds, passes=max(1, int(fnum(self.v_fpasses, 2))),
                              acc=max(0.5, fnum(self.v_facc, 10)), edge_deg=max(0.0, fnum(self.v_fedge, 5)),
                              breakaway=self.v_fbreak.get(), tff_max=min(abs(fnum(self.v_ftmax, 0.8)), 3.0),
                              tff_rate=max(0.01, fnum(self.v_frate, 0.1)))

    def friction_run(self):
        def go():
            if self.ft and not self.ft.done:
                raise SafetyTrip("Friction test already running.")
            self.c.motion_allowed()
            ft = FrictionTest(self.c, self._fparams())
            lo, hi = self.c.limits_rad()
            need = ft.required_range_deg()
            if math.degrees(hi - lo) < need:
                raise SafetyTrip(f"Joint range {math.degrees(hi - lo):.0f}° < needed {need:.0f}°. "
                                 f"Lower the top speed or raise accel.")
            if not messagebox.askyesno("Friction test", "The joint will sweep back and forth between its "
                                                        "limits.\nArea clear and gravity load removed?"):
                return
            self.ft = ft
            self.t_fres.delete("1.0", "end")
            self.ft_thread = threading.Thread(target=ft.run, daemon=True)
            self.ft_thread.start()
        self.call(go)

    def friction_abort(self):
        if self.ft:
            self.ft.abort()

    def friction_save(self):
        if not self.ft or not self.ft.res.points:
            messagebox.showinfo("Save", "No results yet.")
            return
        LOG_DIR.mkdir(exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        r = self.ft.res
        js = LOG_DIR / f"friction_{stamp}.json"
        js.write_text(json.dumps({k: v for k, v in r.__dict__.items() if k != "raw"}, indent=2))
        with open(LOG_DIR / f"friction_{stamp}_raw.csv", "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["vel_rad_s", "torque_Nm"])
            w.writerows(r.raw)
        messagebox.showinfo("Saved", f"{js}\n{js.with_name(js.stem + '_raw.csv')}")

    def _draw_friction(self):
        r = self.ft.res
        ax = self.fax
        ax.clear()
        if r.raw:
            raw = r.raw[:: max(1, len(r.raw) // 2000)]
            ax.plot([x[0] for x in raw], [x[1] for x in raw], ".", ms=2, alpha=0.3, color="#888")
        if r.points:
            ax.errorbar([p[1] for p in r.points], [p[2] for p in r.points], yerr=[p[3] for p in r.points],
                        fmt="o", ms=5, color="#1565c0", label="mean")
        if not math.isnan(r.fc):
            vmax = max(abs(p[1]) for p in r.points) * 1.1
            for s in (1, -1):
                xs = [s * 0.01, s * vmax]
                ax.plot(xs, [r.fc * s + r.b * x + r.offset for x in xs], "-", color="#c62828")
        for fs in (r.fs_pos, r.fs_neg):
            if not math.isnan(fs):
                ax.plot([0], [fs], "^", color="#2e7d32", ms=8)
        ax.axhline(0, color="k", lw=0.5)
        ax.axvline(0, color="k", lw=0.5)
        ax.set_xlabel("ω [rad/s]")
        ax.set_ylabel("τ [Nm]")
        ax.grid(alpha=0.3)
        self.ffig.tight_layout()
        self.fcanvas.draw_idle()

    # ================================================================== tuning actions
    def _sparams(self):
        return StepParams(mode=self.v_smode.get(), step_deg=fnum(self.v_sstep, 10.0),
                          reps=max(1, int(fnum(self.v_sreps, 3))), dwell=max(0.3, fnum(self.v_sdwell, 1.5)),
                          band_pct=max(0.1, fnum(self.v_sband, 2.0)),
                          kp=max(0.0, fnum(self.v_skp, 10.0)), kd=max(0.0, fnum(self.v_skd, 0.5)),
                          ramp=max(0.0, fnum(self.v_sramp, 0.0)), vmax=max(0.05, fnum(self.v_svmax, 3.0)),
                          acc=max(0.1, fnum(self.v_sacc, 20.0)), cur=max(0.1, min(11.0, fnum(self.v_scur, 2.0))))

    def _busy(self):
        return (self.ft and not self.ft.done) or (self.st and not self.st.done)

    def step_run(self):
        def go():
            if self._busy():
                raise SafetyTrip("A test is already running.")
            self.c.motion_allowed()
            prm = self._sparams()
            if abs(prm.step_deg) < 0.5:
                raise ValueError("step must be at least 0.5 deg")
            if prm.mode == "mit" and prm.kp > self.c.cfg.mit_kp_max:
                raise SafetyTrip(f"Kp {prm.kp} above the configured max {self.c.cfg.mit_kp_max}")
            st = StepTest(self.c, prm)
            a, b = st.plan()
            if not messagebox.askyesno("Step test", f"{STEP_MODES[prm.mode]}: steps between "
                                       f"{math.degrees(a):+.1f}° and {math.degrees(b):+.1f}°, "
                                       f"{2 * prm.reps} steps of {prm.dwell} s.\nArea clear?"):
                return
            self.st = st
            self.t_sres.delete("1.0", "end")
            threading.Thread(target=st.run, daemon=True).start()
        self.call(go)

    def step_save(self):
        if not self.st or not self.st.res.metrics:
            messagebox.showinfo("Save", "No results yet.")
            return
        LOG_DIR.mkdir(exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        r = self.st.res
        js = LOG_DIR / f"step_{r.params['mode']}_{stamp}.json"
        js.write_text(json.dumps({"params": r.params, "gains": self._gains_read,
                                  "metrics": [m.__dict__ for m in r.metrics], "averages": r.averages()}, indent=2))
        with open(js.with_suffix(".csv"), "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["step", "t_s", "pos_deg", "vel_rad_s", "torque_Nm", "target_deg"])
            for i, (tr, m) in enumerate(zip(r.traces, r.metrics), 1):
                for t, pos, vel, tq in tr:
                    w.writerow([i, f"{t:.4f}", f"{math.degrees(pos):.4f}", f"{vel:.4f}", f"{tq:.4f}",
                                f"{m.target_deg:.3f}"])
        messagebox.showinfo("Saved", f"{js}\n{js.with_suffix('.csv')}")

    def _draw_step(self):
        r = self.st.res
        ay, at = self.sax_y, self.sax_t
        ay.clear()
        at.clear()
        band = r.params.get("band_pct", 2.0) / 100
        for tr, m in zip(r.traces, r.metrics):
            if not tr:
                continue
            d = m.target_deg - m.start_deg
            t = [x[0] for x in tr]
            ay.plot(t, [(math.degrees(x[1]) - m.start_deg) / d for x in tr], lw=1,
                    color="#1565c0" if m.direction > 0 else "#c62828")
            at.plot(t, [x[3] for x in tr], lw=0.8, color="#1565c0" if m.direction > 0 else "#c62828")
        for y in (1 - band, 1 + band):
            ay.axhline(y, color="#888", lw=0.6, ls=":")
        ay.axhline(1, color="k", lw=0.6)
        ay.set_ylabel("normalized")
        ay.set_title(f"{STEP_MODES[r.params['mode']]}  (blue up, red down)", fontsize=8)
        at.set_ylabel("τ Nm")
        at.set_xlabel("t since step [s]")
        for ax in (ay, at):
            ax.grid(alpha=0.3)
        self.sfig.tight_layout()
        self.scanvas.draw_idle()

    def gains_read(self):
        def go():
            if self._busy():
                raise SafetyTrip("A test is running.")
            self._gains_read = read_loop_gains(self.m)
            for k, v in self._gains_read.items():
                self.v_gain[k].set(f"{v:.5g}")
            self.c.log("loop gains: " + ", ".join(f"{k} {v:.4g}" for k, v in self._gains_read.items()))
        self.call(go)

    def gains_write(self):
        def go():
            if self._busy():
                raise SafetyTrip("A test is running.")
            if not self._gains_read:
                raise SafetyTrip("Press Read first (so the original values can be restored).")
            new = {k: float(self.v_gain[k].get()) for k in LOOP_GAINS}
            big = [k for k, v in new.items() if v > 3 * self._gains_read[k] or v < self._gains_read[k] / 3]
            if big and not messagebox.askyesno("Loop gains", "More than 3x change from the read value:\n"
                                               + ", ".join(big) + "\nWrite anyway?"):
                return
            write_loop_gains(self.m, new)
            back = read_loop_gains(self.m)
            self.c.log("loop gains written (RAM): " + ", ".join(f"{k} {v:.4g}" for k, v in back.items()))
            for k, v in back.items():
                self.v_gain[k].set(f"{v:.5g}")
        self.call(go)

    def gains_restore(self):
        if not self._gains_read:
            messagebox.showinfo("Restore", "Nothing read yet.")
            return
        for k, v in self._gains_read.items():
            self.v_gain[k].set(f"{v:.5g}")
        self.gains_write()

    def gains_flash(self):
        def go():
            if self.c.mode != "idle" or self._busy():
                raise SafetyTrip("STOP the motor first.")
            if not messagebox.askyesno("Save to flash", "This saves ALL current motor parameters (not only the "
                                       "gains) to flash; they survive power-off.\nContinue?"):
                return
            self.m.save_params()
            time.sleep(0.2)
            self.c.log("parameters saved to motor flash")
        self.call(go)

    # ================================================================== recording
    def toggle_record(self):
        if self.v_rec.get():
            if not self.c:
                self.v_rec.set(False)
                return
            LOG_DIR.mkdir(exist_ok=True)
            path = LOG_DIR / f"gui_{time.strftime('%Y%m%d_%H%M%S')}.csv"
            self.rec_file = open(path, "w", newline="")
            w = csv.writer(self.rec_file)
            w.writerow(["t_s", "mode", "pos_deg", "pos_sp_deg", "vel_rad_s", "vel_sp_rad_s", "torque_Nm",
                        "iq_A", "vbus_V", "temp_C", "faults"])
            c = self.c

            def rec(row, fb):
                w.writerow([f"{row[0]:.4f}", c.mode, f"{math.degrees(row[1]):.3f}", f"{math.degrees(row[2]):.3f}",
                            f"{row[3]:.4f}", f"{row[4]:.4f}", f"{row[5]:.4f}", f"{row[6]:.4f}",
                            f"{c.m.s.vbus:.2f}", f"{fb.temp:.1f}", fb.faults])
            c.recorder = rec
            c.log(f"recording {path.name}")
        else:
            self.stop_record()

    def stop_record(self):
        if self.c:
            self.c.recorder = None
        if self.rec_file:
            self.rec_file.close()
            self.rec_file = None
        self.v_rec.set(False)

    # ================================================================== periodic refresh
    def refresh(self):
        try:
            self._refresh()
        finally:
            self.after(100, self.refresh)

    def _refresh(self):
        c = self.c
        if not c or not c.fb:
            return
        fb, s = c.fb, c.m.s
        age = time.monotonic() - fb.t
        self.ro["pos"].config(text=f"{math.degrees(fb.pos):+8.2f} °")
        last = sorted([r[3] for r in list(c.hist)[-VEL_FILTER_N:]] or [fb.vel])
        vel = last[len(last) // 2]                 # median of the last frames (see VEL_FILTER_N)
        self.ro["vel"].config(text=f"{vel:+7.3f} rad/s")
        self.ro["rpm"].config(text=f"{vel * RPM:+7.1f} rpm")
        self.ro["tq"].config(text=f"{fb.torque:+6.3f} Nm")
        self.ro["iq"].config(text=f"{s.iq:+6.3f} A")
        self.ro["vbus"].config(text=f"{s.vbus:5.1f} V")
        self.ro["temp"].config(text=f"{fb.temp:4.1f} °C")
        self.ro["state"].config(text=STATE_NAMES.get(fb.state, "?"))
        self.ro["flt"].config(text=fault_str(fb.faults), foreground="#b00020" if fb.faults else "")
        self.ro["hz"].config(text=f"{s.fb_hz:5.0f} Hz" + ("  (STALE)" if age > 0.3 else ""))
        colors = {"idle": "#555", "pos": "#1565c0", "vel": "#2e7d32", "mit": "#6a1b9a"}
        self.l_mode.config(text=f"MODE: {c.mode.upper()}", foreground=colors.get(c.mode, "#000"))
        self.l_trip.config(text=c.trip_reason)

        lo, hi = c.limits_rad()
        lo_d, hi_d = math.degrees(lo), math.degrees(hi)
        cfg = c.cfg
        if cfg.free_shaft:
            self.l_limits.config(text="FREE SHAFT - limits off", style="Warn.TLabel")
        elif cfg.limits_taught:
            self.l_limits.config(text=f"limits [{lo_d:+.1f}°, {hi_d:+.1f}°]", style="TLabel")
        else:
            self.l_limits.config(text="limits NOT taught → motion refused (tab 1)", style="Warn.TLabel")
        if self._last_lim != (lo_d, hi_d):
            self._last_lim = (lo_d, hi_d)
            self.s_ptarget.config(from_=lo_d, to=hi_d)
            self.s_mtarget.config(from_=lo_d, to=hi_d)
            self.ln_lo.set_ydata([lo_d])
            self.ln_hi.set_ydata([hi_d])
        self._draw_range(fb.pos, lo, hi)

        # STM gateway: which CAN channel the motor answers on + link problems
        if hasattr(self.bus, "channel_of"):
            ch = self.bus.channel_of(self.m.id)
            g, h = self.bus.gw_stats, self.bus.health
            warn = []
            if self.bus.conflicts:
                warn.append(f"ID conflict {sorted(self.bus.conflicts)}")
            if g and (g["wd_trips"] or sum(g["tx_drop"])):
                warn.append(f"wd stops {g['wd_trips']} dropped {sum(g['tx_drop'])}")
            if h and not all(b["init_ok"] for b in h["bus"]):
                warn.append("MCP2515 not up: " + ",".join(f"CAN{i + 1}" for i, b in enumerate(h["bus"])
                                                          if not b["init_ok"]))
            self.l_conn.config(text=f"connected via STM {self.v_port.get()}: motor 0x{self.m.id:02X} on "
                                    f"{'CAN' + str(ch + 1) if ch in (0, 1) else '? (not heard yet)' if ch is None else 'BOTH'}"
                                    + ("   " + " | ".join(warn) if warn else ""))

        # tab infos
        if c.mode == "pos":
            err = math.degrees(fb.pos - c.pos_target)
            self.l_pinfo.config(text=f"target {math.degrees(c.pos_target):+.1f}°   error {err:+.2f}°   "
                                     f"iq {s.iq:+.2f} A")
        else:
            self.l_pinfo.config(text="(disabled)")
        if c.mode == "vel":
            self.l_vinfo.config(text=f"cmd {c.vel_target:+.2f}  sent {c.vel_sent or 0:+.2f}  meas {fb.vel:+.2f} rad/s"
                                     + ("   LIMIT GUARD" if c.vel_limited else ""))
        else:
            self.l_vinfo.config(text="(disabled)")
        if c.mode == "mit":
            tau = c.mit_kp * (c.mit_p_des - fb.pos) + c.mit_kd * (0 - fb.vel) + c.mit_tff
            k_deg = c.mit_kp * math.pi / 180
            self.l_minfo.config(text=f"θd {math.degrees(c.mit_p_des):+.1f}°  θ {math.degrees(fb.pos):+.1f}°\n"
                                     f"τ model {tau:+.3f} Nm   τ meas {fb.torque:+.3f} Nm\n"
                                     f"stiffness {k_deg:.3f} Nm/°"
                                     + ("\nVIRTUAL WALL" if c.mit_wall else "")
                                     + ("\nOVERSPEED BRAKE" if c.mit_overspeed else ""))
        else:
            self.l_minfo.config(text="(stopped)")

        # friction
        fr_lo, fr_hi = c.limits_rad()
        try:
            need = FrictionTest(c, self._fparams()).required_range_deg()
            self.l_frange.config(text=f"needs ≈{need:.0f}° of range, joint has {math.degrees(fr_hi - fr_lo):.0f}°")
        except ValueError:
            pass
        if self.ft:
            self.pb_f["value"] = 100 * self.ft.progress
            self.l_fstat.config(text=self.ft.status)
            if self.ft.done and not getattr(self.ft, "_shown", False):
                self.ft._shown = True
                self.t_fres.delete("1.0", "end")
                self.t_fres.insert("end", f"drive: {FRICTION_DRIVES[self.ft.p.drive]}\n" + self.ft.res.text())
                if self.ft.res.points:
                    self.t_fres.insert("end", "\n\n  v_cmd   v_meas   tau_mean  sd     n\n")
                    for p in self.ft.res.points:
                        self.t_fres.insert("end", f"  {p[0]:+5.2f}  {p[1]:+6.3f}  {p[2]:+7.4f}  {p[3]:.4f}  {p[4]}\n")
                self._draw_friction()

        # step test
        if self.st:
            self.pb_s["value"] = 100 * self.st.progress
            self.l_sstat.config(text=self.st.status)
            if self.st.done and not getattr(self.st, "_shown", False):
                self.st._shown = True
                self.t_sres.delete("1.0", "end")
                self.t_sres.insert("end", self.st.res.text())
                if self.st.res.traces:
                    self._draw_step()

        # events
        ev = list(c.events)
        if len(ev) != self._events_shown or (ev and self.lb_events.get("end") != ev[-1]):
            self.lb_events.delete(0, "end")
            for e in ev[-60:]:
                self.lb_events.insert("end", e)
            self.lb_events.see("end")
            self._events_shown = len(ev)

        self._plot_tick += 1
        if self._plot_tick % 2 == 0:
            self._draw_plot()

    def _draw_range(self, pos, lo, hi):
        cv = self.range_cv
        cv.delete("all")
        w = max(cv.winfo_width(), 200)
        h = 70
        span = hi - lo
        a, b = lo - 0.25 * span, hi + 0.25 * span
        if self.c.cfg.free_shaft:
            a, b = pos - math.pi, pos + math.pi
        x = lambda r: 10 + (w - 20) * (r - a) / (b - a)  # noqa: E731
        cv.create_rectangle(10, 25, w - 10, 45, fill="#ffcdd2", outline="")
        if not self.c.cfg.free_shaft:
            cv.create_rectangle(x(lo), 25, x(hi), 45, fill="#c8e6c9", outline="")
            for r, t in ((lo, f"{math.degrees(lo):+.0f}°"), (hi, f"{math.degrees(hi):+.0f}°")):
                cv.create_line(x(r), 20, x(r), 50, fill="#c62828", width=2)
                cv.create_text(x(r), 58, text=t, font=("Segoe UI", 8))
        if a < 0 < b:
            cv.create_line(x(0), 22, x(0), 48, fill="#555", dash=(2, 2))
            cv.create_text(x(0), 14, text="0", font=("Segoe UI", 8))
        sp, _ = self.c.setpoints()
        if not math.isnan(sp):
            cv.create_line(x(sp), 25, x(sp), 45, fill="#1565c0", width=3)
        px = min(max(x(pos), 4), w - 4)
        cv.create_polygon(px - 7, 8, px + 7, 8, px, 24, fill="black")

    def _draw_plot(self):
        hist = list(self.c.hist)
        if len(hist) < 2:
            return
        t_end = hist[-1][0]
        hist = [r for r in hist if r[0] >= t_end - PLOT_WINDOW_S]
        vf = median_filter([r[3] for r in hist])
        step = max(1, len(hist) // 800)
        hist, vf = hist[::step], vf[::step]
        t = [r[0] - t_end for r in hist]
        cols = list(zip(*hist))
        pos, psp = [math.degrees(x) for x in cols[1]], [math.degrees(x) for x in cols[2]]
        self.ln_p.set_data(t, pos)
        self.ln_ps.set_data(t, psp)
        self.ln_vraw.set_data(t, cols[3])
        self.ln_v.set_data(t, vf)
        self.ln_vs.set_data(t, cols[4])
        self.ln_t.set_data(t, cols[5])
        self.ln_i.set_data(t, cols[6])
        autoscale_min_span(self.ax_p, pos + psp, MIN_SPAN["p"])
        autoscale_min_span(self.ax_v, list(cols[3]) + list(cols[4]), MIN_SPAN["v"])
        autoscale_min_span(self.ax_t, list(cols[5]) + list(cols[6]), MIN_SPAN["t"])
        self.ax_t.set_xlim(-PLOT_WINDOW_S, 0)
        self.canvas.draw_idle()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", action="store_true", help="simulated motor (no hardware)")
    ap.add_argument("-p", "--port", default="COM9")
    ap.add_argument("--id", type=lambda x: int(x, 0), default=0x7F)
    ap.add_argument("--host", type=lambda x: int(x, 0), default=0xFD)
    App(ap.parse_args()).mainloop()


if __name__ == "__main__":
    main()
