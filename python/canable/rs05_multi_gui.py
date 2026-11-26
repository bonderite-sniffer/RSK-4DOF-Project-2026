#!/usr/bin/env python3
"""rs05_multi_gui.py - 4 RS05 with different CAN IDs on one CANable: ID check + simultaneous sequences.

    python rs05_multi_gui.py --sim      4 simulated motors (IDs from the plan)
    python rs05_multi_gui.py            real motors

1. Fill the plan table (one row per motor: ID, mode, steps, timing, current, gains). Saved to
   rs05_multi_plan.json.
2. Connect -> Check IDs (each ID answers once, no strangers on the bus, addressed reads work).
   Identify wiggles one motor a few degrees so you can see which joint that ID is.
3. Joint limits per motor: MIN / MAX here (move the joint by hand), or Free (nothing attached).
   Shared with rs05_gui.py (rs05_gui_config.json).
4. RUN. All enabled motors run their sequence at the same time, with every single-motor safety active.
   STOP ALL / Esc stops everything. A report + CSV is written to logs/.
"""
from __future__ import annotations

import argparse
import math
import threading
import time
import tkinter as tk
from collections import deque
from tkinter import messagebox, ttk

import can
import matplotlib
from serial.tools import list_ports

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402

from rs05_can import MODELS, fault_str, open_bus  # noqa: E402
from rs05_controller import SafetyTrip  # noqa: E402
from rs05_multi import (MODES, MotorPlan, MultiRig, MultiTest, TestSettings, check_ids,  # noqa: E402
                        identify, load_plans, save_plans)

SIM_CHANNEL = "rs05multisim"
COLORS = ["#1565c0", "#c62828", "#2e7d32", "#6a1b9a"]
INSET_DEG = 5.0


def _vel_median(c):
    """Median of the last 5 speed samples - hides the +-0.085 rad/s one-encoder-count spikes."""
    v = sorted(x[3] for x in list(c.hist)[-5:]) or [c.fb.vel]
    return v[len(v) // 2]


class Row:
    """Widgets of one motor row in the plan table."""

    FIELDS = [  # attr, header, width, kind
        ("enabled", "On", 3, "chk"), ("name", "Name", 5, "str"), ("motor_id", "ID", 5, "int"),
        ("mode", "Mode", 8, "mode"), ("steps", "Steps  (deg | rad/s for vel)  value[:s], ...", 30, "str"),
        ("dwell", "Dwell s", 5, "float"), ("cycles", "Cycles", 5, "int"), ("delay", "Delay s", 5, "float"),
        ("cur", "I lim A", 5, "float"), ("vmax", "Vmax", 5, "float"), ("acc", "Acc", 5, "float"),
        ("kp", "Kp", 5, "float"), ("kd", "Kd", 5, "float"), ("ramp", "Ramp", 5, "float"),
        ("rel", "Rel", 3, "chk"), ("end", "End", 5, "end"), ("model", "Model", 5, "model"),
    ]

    def __init__(self, app, parent, r: int, plan: MotorPlan):
        self.app, self.vars = app, {}
        for c, (attr, _, w, kind) in enumerate(self.FIELDS):
            val = getattr(plan, attr)
            if kind == "chk":
                v = tk.BooleanVar(value=bool(val))
                ttk.Checkbutton(parent, variable=v).grid(row=r, column=c)
            elif kind in ("mode", "end", "model"):
                v = tk.StringVar(value=val)
                vals = {"mode": MODES, "end": ("hold", "stop"), "model": tuple(MODELS)}[kind]
                ttk.Combobox(parent, textvariable=v, values=vals,
                             width=w, state="readonly").grid(row=r, column=c, padx=1)
            else:
                v = tk.StringVar(value=hex(val) if attr == "motor_id" else str(val))
                ttk.Entry(parent, textvariable=v, width=w).grid(row=r, column=c, padx=1)
            self.vars[attr] = v
        c = len(self.FIELDS)
        self.l_lim = ttk.Label(parent, text="-", width=17)
        self.l_lim.grid(row=r, column=c, padx=3)
        self.v_free = tk.BooleanVar(value=False)
        ttk.Checkbutton(parent, text="Free", variable=self.v_free, command=self.free_changed).grid(row=r, column=c + 1)
        b = ttk.Frame(parent)
        b.grid(row=r, column=c + 2, sticky="w")
        for txt, cmd in (("MIN", lambda: self.teach("min")), ("MAX", lambda: self.teach("max")),
                         ("Zero", self.zero), ("Identify", self.identify)):
            ttk.Button(b, text=txt, width=len(txt) + 1, command=cmd).pack(side="left")
        self.taught = set()

    def plan(self) -> MotorPlan:
        kw = {}
        for attr, _, _, kind in self.FIELDS:
            s = self.vars[attr].get()
            try:
                if kind == "int":
                    kw[attr] = int(str(s), 0)
                elif kind == "float":
                    kw[attr] = float(s)
                else:
                    kw[attr] = s
            except ValueError:
                raise ValueError(f"row {self.vars['name'].get()}: '{s}' is not a valid {attr}")
        return MotorPlan(**kw)

    @property
    def ctrl(self):
        rig = self.app.rig
        try:
            return rig.ctrls.get(int(self.vars["motor_id"].get(), 0)) if rig else None
        except ValueError:
            return None

    def _need(self):
        c = self.ctrl
        if c is None:
            raise SafetyTrip("Motor not connected (Connect with this row enabled).")
        if self.app.test and self.app.test.running:
            raise SafetyTrip("A test is running.")
        return c

    def free_changed(self):
        c = self.ctrl
        if c is None:
            return
        if self.v_free.get() and not messagebox.askyesno(
                "Free shaft", f"{self.vars['name'].get()}: disable joint limits? Only if NOTHING is attached."):
            self.v_free.set(False)
            return
        c.cfg.free_shaft = self.v_free.get()
        self.app.rig.save_configs()

    def teach(self, which):
        try:
            c = self._need()
            if c.mode != "idle":
                raise SafetyTrip("STOP first; teach with the motor limp.")
            deg = c.teach(which)
            if which == "min":
                c.cfg.lim_min_deg = deg + INSET_DEG
            else:
                c.cfg.lim_max_deg = deg - INSET_DEG
            self.taught.add(which)
            if self.taught >= {"min", "max"} or c.cfg.limits_taught:
                c.set_limits(c.cfg.lim_min_deg, c.cfg.lim_max_deg)
            self.app.rig.save_configs()
            self.app.log(f"{self.vars['name'].get()}: {which} end {deg:+.1f} deg -> limit "
                         f"{c.cfg.lim_min_deg if which == 'min' else c.cfg.lim_max_deg:+.1f}")
        except Exception as e:  # noqa: BLE001
            messagebox.showerror("Teach", str(e))

    def zero(self):
        try:
            c = self._need()
            if not messagebox.askyesno("Zero", f"{self.vars['name'].get()}: current position becomes 0?"):
                return
            old, new, ok = c.zero_here(False)
            self.app.rig.save_configs()
            self.app.log(f"{self.vars['name'].get()}: zero {'OK' if ok else 'FAILED'} "
                         f"({math.degrees(old):+.1f} -> {math.degrees(new):+.1f} deg)")
        except Exception as e:  # noqa: BLE001
            messagebox.showerror("Zero", str(e))

    def identify(self):
        try:
            c = self._need()
            c.motion_allowed()
        except Exception as e:  # noqa: BLE001
            messagebox.showerror("Identify", str(e))
            return
        name = self.vars["name"].get()
        self.app.log(f"{name}: identify wiggle on ID {self.vars['motor_id'].get()}")

        def go():
            try:
                identify(c)
            except Exception as e:  # noqa: BLE001
                self.app.log(f"{name}: identify failed: {e}")
        threading.Thread(target=go, daemon=True).start()


class MultiApp(tk.Tk):
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.title("RS05 multi-motor test - 4 IDs, one CANable")
        sw, sh = self.winfo_screenwidth(), self.winfo_screenheight()
        self.geometry(f"{min(1600, sw - 40)}x{min(980, sh - 80)}+10+10")
        self.minsize(1200, 760)
        self.bus = self.rig = self.test = None
        self.sims = []
        self.stm_emu = None
        self._events: deque[str] = deque(maxlen=400)
        self._shown_report = None
        self._vbus_hist: deque = deque(maxlen=1500)
        self._t0 = time.monotonic()
        self.plans, self.settings = load_plans()
        while len(self.plans) < 4:
            self.plans.append(MotorPlan(enabled=False, name=f"J{len(self.plans) + 1}", motor_id=len(self.plans) + 1))
        style = ttk.Style(self)
        style.configure("Stop.TButton", foreground="white", background="#c62828", font=("Segoe UI", 11, "bold"))
        style.configure("Run.TButton", font=("Segoe UI", 11, "bold"))
        style.configure("Note.TLabel", foreground="#555")
        self._build()
        self.bind("<Escape>", lambda e: self.stop_all())
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(100, self.refresh)

    # ================================================================== layout
    def _build(self):
        top = ttk.Frame(self, padding=(6, 6, 6, 2))
        top.pack(fill="x")
        ttk.Label(top, text="Interface").pack(side="left")
        self.v_iface = tk.StringVar(value="sim" if self.args.sim else "slcan")
        ttk.Combobox(top, textvariable=self.v_iface, values=("slcan", "gs_usb", "stm", "sim", "stm-sim"), width=8,
                     state="readonly").pack(side="left", padx=3)
        ttk.Label(top, text="Port").pack(side="left")
        self.v_port = tk.StringVar(value=self.args.port)
        self.cb_port = ttk.Combobox(top, textvariable=self.v_port, width=8)
        self.cb_port.pack(side="left", padx=3)
        ports = [p.device for p in list_ports.comports()]
        self.cb_port["values"] = ports
        if ports and self.v_port.get() not in ports and not self.args.sim:
            self.v_port.set(ports[0])
        ttk.Label(top, text="Bitrate").pack(side="left")
        self.v_bitrate = tk.StringVar(value="1000000")
        ttk.Combobox(top, textvariable=self.v_bitrate, values=("1000000", "500000", "250000", "125000"),
                     width=8).pack(side="left", padx=3)
        ttk.Label(top, text="Host").pack(side="left")
        self.v_host = tk.StringVar(value="0xfd")
        ttk.Entry(top, textvariable=self.v_host, width=5).pack(side="left", padx=3)
        self.b_conn = ttk.Button(top, text="Connect", command=self.toggle_connect)
        self.b_conn.pack(side="left", padx=6)
        ttk.Button(top, text="Check IDs", command=self.check_ids).pack(side="left")
        ttk.Button(top, text="STOP ALL  (Esc)", style="Stop.TButton", command=self.stop_all).pack(side="left", padx=12)
        self.l_conn = ttk.Label(top, text="disconnected", style="Note.TLabel")
        self.l_conn.pack(side="left", padx=6)

        # plan table
        pf = ttk.LabelFrame(self, text=" Plan: one row per motor ", padding=4)
        pf.pack(fill="x", padx=6)
        for c, (_, hdr, _, _) in enumerate(Row.FIELDS):
            ttk.Label(pf, text=hdr, style="Note.TLabel").grid(row=0, column=c, sticky="w")
        n = len(Row.FIELDS)
        for c, hdr in enumerate(("joint limits", "", "teach / zero / identify")):
            ttk.Label(pf, text=hdr, style="Note.TLabel").grid(row=0, column=n + c, sticky="w")
        self.rows = [Row(self, pf, i + 1, p) for i, p in enumerate(self.plans[:4])]
        ttk.Label(pf, style="Note.TLabel", justify="left",
                  text="Modes: pos-pp / pos-csp (deg, Vmax rad/s, Acc rad/s²), vel (rad/s, Acc rad/s², Vmax = cap), "
                       "mit (deg, Kp Nm/rad, Kd, Ramp rad/s).  Cycles 0 = until time limit.  Rel = steps relative "
                       "to the start position.  End: hold last step or stop (limp).").grid(
            row=6, column=0, columnspan=n + 3, sticky="w", pady=(4, 0))

        # test settings
        tf = ttk.Frame(self, padding=(6, 4))
        tf.pack(fill="x")
        self.v_dur = tk.StringVar(value=str(self.settings.duration))
        self.v_vbmin = tk.StringVar(value=str(self.settings.vbus_min))
        self.v_stoptrip = tk.BooleanVar(value=self.settings.stop_all_on_trip)
        self.v_record = tk.BooleanVar(value=self.settings.record)
        for lab, var, w in (("Time limit s", self.v_dur, 6), ("Stop all if VBUS < V", self.v_vbmin, 5)):
            ttk.Label(tf, text=lab).pack(side="left")
            ttk.Entry(tf, textvariable=var, width=w).pack(side="left", padx=(2, 10))
        ttk.Checkbutton(tf, text="stop all if one motor trips", variable=self.v_stoptrip).pack(side="left")
        ttk.Checkbutton(tf, text="record CSV + report", variable=self.v_record).pack(side="left", padx=8)
        self.b_run = ttk.Button(tf, text="▶ RUN TEST", style="Run.TButton", command=self.run_test)
        self.b_run.pack(side="left", padx=10)
        ttk.Button(tf, text="Save plan", command=self.save_plan).pack(side="left")
        self.pb = ttk.Progressbar(tf, length=220)
        self.pb.pack(side="left", padx=10)
        self.l_test = ttk.Label(tf, text="", style="Note.TLabel")
        self.l_test.pack(side="left")

        # status + log | plot
        body = ttk.Panedwindow(self, orient="horizontal")
        body.pack(fill="both", expand=True, padx=6, pady=4)
        left = ttk.Frame(body)
        body.add(left, weight=1)
        cols = ("motor", "ID", "fb Hz", "mode", "test", "setpoint", "pos°", "vel", "τ Nm", "iq A", "iq pk",
                "T °C", "VBUS", "gaps", "fault / trip")
        self.tree = ttk.Treeview(left, columns=cols, show="headings", height=4)
        widths = (55, 40, 50, 45, 80, 65, 60, 55, 55, 50, 50, 45, 50, 40, 220)
        for c, w in zip(cols, widths):
            self.tree.heading(c, text=c)
            self.tree.column(c, width=w, anchor="e" if c not in ("motor", "fault / trip", "test", "mode") else "w")
        self.tree.pack(fill="x")
        for i, c in enumerate(COLORS):
            self.tree.tag_configure(f"m{i}", foreground=c)
        self.tree.tag_configure("bad", background="#ffcdd2")
        ttk.Label(left, text="Events / ID check", style="Note.TLabel").pack(anchor="w", pady=(6, 0))
        self.lb = tk.Listbox(left, font=("Consolas", 9))
        self.lb.pack(fill="both", expand=True)

        right = ttk.Frame(body)
        body.add(right, weight=1)
        self.fig = Figure(figsize=(6, 4), dpi=90)
        self.ax_p = self.fig.add_subplot(311)
        self.ax_i = self.fig.add_subplot(312, sharex=self.ax_p)
        self.ax_v = self.fig.add_subplot(313, sharex=self.ax_p)
        self.ax_p.set_ylabel("pos °")
        self.ax_i.set_ylabel("iq A")
        self.ax_v.set_ylabel("VBUS V")
        self.ax_v.set_xlabel("t s")
        self.lines_p, self.lines_i = [], []
        for i, c in enumerate(COLORS):
            self.lines_p.append(self.ax_p.plot([], [], color=c, lw=1, label=f"J{i + 1}")[0])
            self.lines_i.append(self.ax_i.plot([], [], color=c, lw=1)[0])
        self.line_v = self.ax_v.plot([], [], color="#333", lw=1)[0]
        self.ax_p.legend(loc="upper left", fontsize=7, ncol=4)
        self.fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self._plot_t = 0.0

    # ================================================================== helpers
    def log(self, text):
        self._events.append(time.strftime("%H:%M:%S ") + text)

    def read_plans(self) -> list[MotorPlan]:
        return [r.plan() for r in self.rows]

    def read_settings(self) -> TestSettings:
        return TestSettings(duration=float(self.v_dur.get()), vbus_min=float(self.v_vbmin.get()),
                            stop_all_on_trip=self.v_stoptrip.get(), record=self.v_record.get())

    def save_plan(self):
        try:
            save_plans(self.read_plans(), self.read_settings())
            self.log("plan saved (rs05_multi_plan.json)")
        except ValueError as e:
            messagebox.showerror("Plan", str(e))

    # ================================================================== connect
    def toggle_connect(self):
        if self.rig:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        try:
            plans = self.read_plans()
            ids = [p.motor_id for p in plans if p.enabled]
            if not ids:
                raise ValueError("Enable at least one row.")
            if len(set(ids)) != len(ids):
                raise ValueError("Two enabled rows have the same ID.")
            host = int(self.v_host.get(), 0)
            if self.v_iface.get() == "sim":
                from rs05_sim import FakeRS05
                if not self.sims:
                    self.sims = [FakeRS05(SIM_CHANNEL, i, host) for i in ids]
                self.bus = can.Bus(interface="virtual", channel=SIM_CHANNEL)
            elif self.v_iface.get() == "stm-sim":
                from rs05_stm import StmBus
                from rs05_stm_emu import StmEmulator
                if self.stm_emu is None:            # first half of the IDs on CAN1, the rest on CAN2
                    self.stm_emu = StmEmulator()
                    half = (len(ids) + 1) // 2
                    for k, i in enumerate(ids):
                        self.stm_emu.add_motor(0 if k < half else 1, i)
                self.bus = StmBus("stm-sim", serial_port=self.stm_emu.connect())
            else:
                if self.v_iface.get() == "stm":
                    self.v_bitrate.set("1000000")   # STM firmware: 1 Mbit/s fixed
                self.bus = open_bus(self.v_iface.get(), self.v_port.get(), int(self.v_bitrate.get()))
            self.rig = MultiRig(self.bus, ids, host, models={p.motor_id: p.model for p in plans if p.enabled})
        except Exception as e:  # noqa: BLE001
            self._teardown()
            messagebox.showerror("Connect failed", str(e))
            return
        if self.v_iface.get() in ("sim", "stm-sim"):
            for c in self.rig.ctrls.values():
                if not (c.cfg.limits_taught or c.cfg.free_shaft):
                    c.set_limits(-90, 90)
            self.log("sim: untaught motors got limits [-90, 90] deg (sim hard stops at +-120)")
        for r in self.rows:
            c = r.ctrl
            if c:
                r.v_free.set(c.cfg.free_shaft)
        self.b_conn.config(text="Disconnect")
        self.l_conn.config(text=f"connected {self.v_iface.get()} {self.v_port.get()}: "
                                + ", ".join(f"0x{i:02X}" for i in ids))
        if self.rig.missing:
            self.log("NO FEEDBACK from " + ", ".join(f"0x{i:02X}" for i in self.rig.missing)
                     + " - check ID / cable / power")
            messagebox.showwarning("Connect", "No feedback from " + ", ".join(f"0x{i:02X}" for i in self.rig.missing)
                                   + ".\nRun Check IDs; check that motor's cable, connector and power.")
        self.log("connected: " + ", ".join(f"0x{i:02X}" for i in ids))
        if hasattr(self.bus, "stats_text"):
            self.after(1500, self._log_stm_stats)

    def _log_stm_stats(self):
        if self.bus is not None and hasattr(self.bus, "stats_text"):
            for line in self.bus.stats_text().splitlines():
                self.log(line)

    def _teardown(self):
        if self.test and self.test.running:
            self.test.abort("disconnect")
            self.test.wait(2)
        if self.rig:
            self.rig.save_configs()
            self.rig.close()
        if self.bus:
            self.bus.shutdown()
        self.rig = self.bus = None

    def disconnect(self):
        self._teardown()
        self.b_conn.config(text="Connect")
        self.l_conn.config(text="disconnected")

    # ================================================================== actions
    def check_ids(self):
        if not self.rig:
            messagebox.showwarning("Check IDs", "Connect first.")
            return
        if self.test and self.test.running:
            return
        ids = [p.motor_id for p in self.read_plans() if p.enabled]
        self.log("ID check started")

        def go():
            ok, lines = check_ids(self.rig, ids)
            for ln in lines:
                self.log("  " + ln)
            self.log(f"ID check: {'PASS' if ok else 'FAIL'}")
            self.after(0, lambda: (messagebox.showinfo if ok else messagebox.showwarning)(
                "Check IDs", ("PASS\n\n" if ok else "FAIL\n\n") + "\n".join(lines)
                + "\n\nUse Identify on each row to confirm which physical joint each ID drives."))
        threading.Thread(target=go, daemon=True).start()

    def run_test(self):
        if not self.rig:
            messagebox.showwarning("Run", "Connect first.")
            return
        if self.test and self.test.running:
            return
        try:
            plans, st = self.read_plans(), self.read_settings()
            save_plans(plans, st)
            test = MultiTest(self.rig, plans, st, log=self.log)
        except Exception as e:  # noqa: BLE001
            messagebox.showerror("Cannot start", str(e))
            return
        lines = [f"{r.plan.name} 0x{r.plan.motor_id:02X} {r.plan.mode}: [{r.plan.steps}] {r.plan.unit()}, "
                 f"cycles {r.plan.cycles or '∞'}, delay {r.plan.delay}s, I {r.plan.cur} A" for r in test.runs]
        itot = sum(r.plan.cur for r in test.runs if r.plan.mode != "mit")
        if not messagebox.askyesno(
                "Run test", "\n".join(lines) + f"\n\nTime limit {st.duration:.0f} s.  Sum of current limits "
                f"{itot:.1f} A (phase current, MIT not limited).\nAll joints clear? PSU limit set? Hand on the switch?"):
            return
        self._shown_report = None
        self.test = test
        test.start()

    def stop_all(self):
        if self.test and self.test.running:
            self.test.abort("STOP ALL")
        if self.rig:
            self.rig.stop_all()
        self.log("STOP ALL")

    # ================================================================== refresh
    def refresh(self):
        try:
            self._refresh()
        except Exception as e:  # noqa: BLE001
            self.log(f"GUI refresh error: {e}")
        self.after(100, self.refresh)

    def _refresh(self):
        while self._events:
            self.lb.insert("end", self._events.popleft())
            self.lb.see("end")
        runs = {r.plan.motor_id: r for r in self.test.runs} if self.test else {}
        self.tree.delete(*self.tree.get_children())
        vbs = []
        for i, row in enumerate(self.rows):
            c = row.ctrl
            name = row.vars["name"].get()
            if c is None:
                row.l_lim.config(text="-")
                self.tree.insert("", "end", values=(name, row.vars["motor_id"].get()) + ("",) * 12 + ("not connected",))
                continue
            cfg, fb, ms = c.cfg, c.fb, c.m.s
            row.l_lim.config(text="FREE" if cfg.free_shaft else (
                f"[{cfg.lim_min_deg:+.0f}, {cfg.lim_max_deg:+.0f}]°" if cfg.limits_taught else "NOT taught"))
            r = runs.get(c.m.id)
            live = fb is not None and time.monotonic() - fb.t < 0.3
            if not math.isnan(ms.vbus):
                vbs.append(ms.vbus)
            sp = ""
            if r and not math.isnan(r.target):
                sp = f"{r.target:+.2f}" if r.plan.mode == "vel" else f"{math.degrees(r.target):+.1f}"
            trip = c.trip_reason or (r.note if r else "")
            flt = fault_str(fb.faults) if fb and fb.faults else ""
            vals = (name, f"0x{c.m.id:02X}", f"{ms.fb_hz:.0f}" if live else "LOST", c.mode,
                    f"{r.state} s{r.step} c{r.cycle}" if r else "", sp,
                    f"{math.degrees(fb.pos):+.1f}" if fb else "", f"{_vel_median(c):+.2f}" if fb else "",
                    f"{fb.torque:+.2f}" if fb else "", f"{ms.iq:+.2f}", f"{r.stats.iq_peak:.2f}" if r else "",
                    f"{fb.temp:.1f}" if fb else "", f"{ms.vbus:.1f}", f"{r.stats.gaps}" if r else "",
                    " ".join(x for x in (flt, trip) if x))
            tags = (f"m{i}",) + (("bad",) if (not live or flt or trip) else ())
            self.tree.insert("", "end", values=vals, tags=tags)
        now = time.monotonic() - self._t0
        if vbs:
            self._vbus_hist.append((now, min(vbs)))

        t = self.test
        if t:
            if t.running:
                self.pb["value"] = 100 * min(1.0, t.elapsed / t.settings.duration)
                self.l_test.config(text=f"running {t.elapsed:.0f} / {t.settings.duration:.0f} s")
            elif self._shown_report is not t:
                self._shown_report = t
                self.pb["value"] = 0
                self.l_test.config(text=f"ended: {t.end_reason}")
                self.show_report(t.summary() + (f"\n\nreport: {t.report_path}" if t.report_path else ""))

        if now - self._plot_t > 0.4 and self.rig:
            self._plot_t = now
            self._draw()

    def _draw(self):
        tmin = None
        for i, row in enumerate(self.rows):
            c = row.ctrl
            if c is None or not c.hist:
                self.lines_p[i].set_data([], [])
                self.lines_i[i].set_data([], [])
                continue
            h = list(c.hist)[-1500:]
            ts = [x[0] + (c._t0 - self._t0) for x in h]
            self.lines_p[i].set_data(ts, [math.degrees(x[1]) for x in h])
            self.lines_i[i].set_data(ts, [x[6] for x in h])
            self.lines_p[i].set_label(row.vars["name"].get())
            tmin = ts[0] if tmin is None else min(tmin, ts[0])
        if self._vbus_hist:
            vt, vv = zip(*self._vbus_hist)
            self.line_v.set_data(vt, vv)
        for ax in (self.ax_p, self.ax_i, self.ax_v):
            ax.relim()
            ax.autoscale_view()
        self.ax_p.legend(loc="upper left", fontsize=7, ncol=4)
        self.canvas.draw_idle()

    def show_report(self, text):
        w = tk.Toplevel(self)
        w.title("Test report")
        txt = tk.Text(w, font=("Consolas", 9), width=150, height=28)
        txt.pack(fill="both", expand=True)
        txt.insert("end", text)
        ttk.Button(w, text="Close", command=w.destroy).pack(pady=4)

    def on_close(self):
        try:
            save_plans(self.read_plans(), self.read_settings())
        except ValueError:
            pass
        self._teardown()
        for s in self.sims:
            s.close()
        if self.stm_emu:
            self.stm_emu.close()
        self.destroy()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("-p", "--port", default="COM9")
    a = ap.parse_args()
    MultiApp(a).mainloop()


if __name__ == "__main__":
    main()
