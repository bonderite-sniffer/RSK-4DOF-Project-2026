#!/usr/bin/env python3
"""
rs05_tests.py - bench tests for 4x RobStride EduLite 05 (2 per CAN bus) via the
STM32H723 HAL firmware. Nothing moves unless you run a command.

Motors: M1, M2 on CAN1 / M3, M4 on CAN2. Select with  -m 1 2  or  -m all.

  python rs05_tests.py -p COM8 info                       # ping, ID scan, CAN health, VBUS
  python rs05_tests.py -p COM8 monitor                    # live numbers (motors idle)
  python rs05_tests.py -p COM8 setid --motor 2 --new 2    # change CAN ID (ONE motor on the bus!)
  python rs05_tests.py -p COM8 zero -m all                # current pose = 0 (calib.c)
  python rs05_tests.py -p COM8 home -m 1 --dir -1         # hard-stop homing (calib.c)
  python rs05_tests.py -p COM8 pos -m 1 2                 # 90,-90,45,-45 deg
  python rs05_tests.py -p COM8 vel-sweep -m 1 --rpm 20 300 20 --i-understand-rotation
  python rs05_tests.py -p COM8 torque -m 1 --steps 0.2 0.4
  python rs05_tests.py -p COM8 mit -m 1 --kp 5 --kd 0.3
  python rs05_tests.py -p COM8 health
Live plot: python live_plot.py -p COM8
Ctrl+C = E-STOP. If this script dies, the firmware watchdog stops the motors.
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import statistics as stats
import sys
import time

from rs05_host import MODE_NAMES, P, RPM_PER_RADS, RS05Link, fault_str, mask


def logpath(name):
    os.makedirs("logs", exist_ok=True)
    return os.path.join("logs", f"{name}_{time.strftime('%Y%m%d_%H%M%S')}.csv")


def motors_arg(vals):
    if vals in (None, ["all"]):
        return [1, 2, 3, 4]
    return sorted({int(v) for v in vals})


def setup(L, a):
    info = L.ping()
    print(f"firmware {info['fw']} (protocol {info['proto']}), loop {info['ctrl_hz']} Hz, "
          f"{info['n_motors']} motors on {info['n_bus']} buses")
    L.set_cfg("TSCALE", a.tscale)
    L.set_cfg("POS_MIN", math.radians(a.soft_min))
    L.set_cfg("POS_MAX", math.radians(a.soft_max))
    L.set_cfg("SOFTLIM_EN", 1)
    L.set_cfg("TEMP_MAX", a.temp_max)
    for k, cid in enumerate(a.ids, start=1):
        L.set_motor_id(k, cid)
    L.wait_telem(2.0)


def require_online(L, motors):
    time.sleep(0.2)
    bad = [k for k in motors if not L.latest.m[k - 1].online]
    if bad:
        sys.exit(f"motor(s) {bad} offline. Run 'info' (scan) and check wiring / CAN IDs (--ids).")


def wait_until(L, cond, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond(L.latest):
            return True
        time.sleep(0.01)
    return False


def summarize(rows, k):
    i = k - 1
    col = lambda f: [f(t.m[i]) for t in rows]
    v, tq, iq, tmp = col(lambda m: m.vel), col(lambda m: m.torque), col(lambda m: m.iq), col(lambda m: m.temp)
    return dict(vel=stats.fmean(v), vel_sd=stats.pstdev(v), torque=stats.fmean(tq),
                torque_absmax=max(abs(x) for x in tq), iq=stats.fmean(iq),
                iq_absmax=max(abs(x) for x in iq), temp=max(tmp))


# ================================================================ commands
def cmd_info(L, a):
    for bus, r in L.scan(3).items():
        st = "BUS ERROR: nobody ACKs (crystal 16 MHz? termination? 48 V?)" if r["bus_error"] else "ok"
        print(f"{bus}: CAN IDs found {r['ids']}  [{st}]")
    print(f"configured: M1,M2 on CAN1 = {a.ids[0]},{a.ids[1]}   M3,M4 on CAN2 = {a.ids[2]},{a.ids[3]}")
    cmd_health(L, a)


def cmd_health(L, a):
    h = L.get_health()
    for b, d in enumerate(h.bus, start=1):
        print(f"CAN{b}: init {'OK' if d['init_ok'] else 'FAIL'}  TEC {d['tec']} REC {d['rec']} EFLG 0x{d['eflg']:02X}  "
              f"tx {d['tx_sent']} fail {d['tx_fail']} busy {d['tx_busy']}  rx {d['rx_count']} ovf {d['rx_ovf']}  "
              f"bus-off {d['bus_off']}  spi-err {d['spi_err']}")
    for k in range(4):
        age = h.fb_age_ms[k]
        print(f"M{k + 1}: VBUS {h.vbus[k]:5.1f} V, last feedback {'>65 s' if age == 65535 else f'{age} ms'} ago")


def cmd_monitor(L, a):
    print("Ctrl+C to quit. Motors idle - move them by hand.")
    while True:
        t = L.latest
        print(" | ".join(f"M{k+1} {MODE_NAMES.get(m.mode,'?')[:4]:4s}{'*' if m.online else ' '} "
                         f"{m.pos_deg:7.1f}deg {m.rpm:6.1f}rpm {m.torque:5.2f}Nm {m.temp:4.1f}C"
                         for k, m in enumerate(t.m)), end="\r")
        time.sleep(0.1)


def cmd_setid(L, a):
    print(f"Changing CAN ID of M{a.motor} (currently {a.ids[a.motor - 1]}) -> {a.new}.")
    print("Only this motor may be on the bus with the old ID. Continue? [y/N] ", end="")
    if input().strip().lower() != "y":
        return
    L.disable(0x0F)
    ok = L.set_can_id(a.motor, a.new)
    print("OK, motor answers on the new ID." if ok else "FAILED: motor did not answer on the new ID.")
    if ok:
        print(f"Use --ids with {a.new} for M{a.motor} from now on. Run 'save' if the ID is lost after power-off.")


def cmd_zero(L, a):
    ms = motors_arg(a.m)
    L.disable(mask(*ms), clear_fault=True)
    print("Motors disabled. Put the arm(s) where 0 deg should be, then press Enter.")
    input()
    res = L.zero_here(mask(*ms))
    time.sleep(0.2)
    for k, ok in res.items():
        print(f"M{k}: {'zero OK' if ok else 'ZERO FAILED'}  pos now {L.latest.m[k-1].pos_deg:.2f} deg")


def cmd_home(L, a):
    ms = motors_arg(a.m)
    require_online(L, ms)
    t0 = time.time()
    L.home(mask(*ms), a.dir, a.speed, a.cur, a.backoff)
    for k in ms:
        r = L.wait_event(1, k, 35, t0)
        print(f"M{k}: {'homed, holding 0' if r else 'homing FAILED / timeout'}")


def cmd_save(L, a):
    L.save_params(mask(*motors_arg(a.m)))
    print("save requested (VERIFY on your motor firmware: power-cycle and check)")


def cmd_pos(L, a):
    ms = motors_arg(a.m)
    require_online(L, ms)
    path = logpath("pos")
    L.start_log(path)
    try:
        for deg in a.targets:
            tgt = math.radians(deg)
            L.tag = f"pos={deg}"
            t0 = time.time()
            L.pos(mask(*ms), tgt, a.vlim, a.cur)
            ok = wait_until(L, lambda t: all(abs(t.m[k-1].pos - tgt) < math.radians(a.tol) for k in ms), 3 + 2 * math.pi / a.vlim)
            rows = L.collect(a.dwell)
            for k in ms:
                s = summarize(rows, k)
                err = math.degrees(stats.fmean(t.m[k-1].pos for t in rows) - tgt)
                print(f"  M{k} -> {deg:6.1f} deg  {'reached' if ok else 'NOT reached'} in {time.time()-t0-a.dwell:4.1f}s  "
                      f"err {err:+.2f} deg  hold {s['torque']:+.3f} Nm  iq {s['iq']:+.2f} A  {s['temp']:.1f} C")
        L.pos(mask(*ms), 0.0, a.vlim, a.cur)
        wait_until(L, lambda t: all(abs(t.m[k-1].pos) < 0.05 for k in ms), 10)
    finally:
        L.tag = ""
        L.stop_log()
        L.disable(mask(*ms))
    print(f"log: {path}")


def cmd_vel_sweep(L, a):
    ms = motors_arg(a.m)
    if not a.i_understand_rotation:
        sys.exit("Velocity sweep spins continuously. Clear the swing area, then add --i-understand-rotation.")
    require_online(L, ms)
    L.set_cfg("SOFTLIM_EN", 0)
    path = logpath("velsweep")
    L.start_log(path)
    summary = []
    try:
        for rpm in range(a.rpm[0], a.rpm[1] + 1, a.rpm[2]):
            w = rpm / RPM_PER_RADS
            L.tag = f"rpm={rpm}"
            L.vel(mask(*ms), w, a.cur, a.acc)
            time.sleep(a.settle)
            rows = L.collect(a.hold)
            stop = False
            for k in ms:
                s = summarize(rows, k)
                sat = abs(s["vel"] - w) > 0.08 * w + 0.3 or s["iq_absmax"] >= 0.97 * a.cur
                summary.append(dict(motor=k, rpm_cmd=rpm, rpm_meas=s["vel"] * RPM_PER_RADS, torque=s["torque"],
                                    torque_absmax=s["torque_absmax"], iq=s["iq"], temp_max=s["temp"], saturated=int(sat)))
                print(f"  M{k} {rpm:4d} rpm -> {s['vel']*RPM_PER_RADS:6.1f}  T {s['torque']:+.3f} (max {s['torque_absmax']:.2f})  "
                      f"iq {s['iq']:+.2f}  {s['temp']:.1f}C {'<-- SATURATED' if sat else ''}")
                stop |= sat or s["temp"] > a.temp_max - 5
            if stop:
                break
    finally:
        L.vel(mask(*ms), 0.0, a.cur, a.acc)
        time.sleep(a.rpm[1] / RPM_PER_RADS / a.acc + 1)
        L.disable(mask(*ms))
        L.set_cfg("SOFTLIM_EN", 1)
        L.tag = ""
        L.stop_log()
    out = path.replace(".csv", "_summary.csv")
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(summary[0]))
        w.writeheader()
        w.writerows(summary)
    print(f"log: {path}\nsummary: {out}")


def cmd_torque(L, a):
    ms = motors_arg(a.m)
    require_online(L, ms)
    print(f"MIT torque steps, kp=0. torque scale = +-{a.tscale} Nm.")
    path = logpath("torque")
    L.start_log(path)
    try:
        for tq in a.steps:
            for sgn in ((1, -1) if a.both_dirs else (1,)):
                L.tag = f"t={sgn*tq:+.2f}"
                L.mit(mask(*ms), 0, 0, 0, a.kd, sgn * tq)
                rows = L.collect(a.duration)
                for k in ms:
                    s = summarize(rows, k)
                    print(f"  M{k} t_cmd {sgn*tq:+.2f}  T_fb {s['torque']:+.3f} (max {s['torque_absmax']:.2f})  iq {s['iq']:+.2f}  {s['temp']:.1f}C")
                L.mit(mask(*ms), 0, 0, 0, a.kd, 0)
                time.sleep(a.rest)
    finally:
        L.tag = ""
        L.stop_log()
        L.disable(mask(*ms))
    print(f"log: {path}")


def cmd_mit(L, a):
    ms = motors_arg(a.m)
    require_online(L, ms)
    for k in ms:   # start at the current position -> no jump
        p = math.radians(a.p) if a.p is not None else L.latest.m[k-1].pos
        L.mit(mask(k), p, a.v, a.kp, a.kd, a.t)
    print(f"MIT tau = {a.kp}*(p-pos) + {a.kd}*({a.v}-vel) + {a.t}  for {a.duration}s")
    try:
        t_end = time.time() + a.duration
        while time.time() < t_end:
            print("  " + "  ".join(f"M{k} {L.latest.m[k-1].pos_deg:7.2f}deg {L.latest.m[k-1].torque:+.3f}Nm" for k in ms), end="\r")
            time.sleep(0.05)
    finally:
        L.disable(mask(*ms))
    print()


def cmd_param(L, a):
    if a.read:
        idx = P.get(a.read) or int(a.read, 0)
        print(f"M{a.motor} 0x{idx:04X} = {L.read_param(a.motor, idx, not a.int)}")
    if a.write:
        idx = P.get(a.write[0]) or int(a.write[0], 0)
        L.write_param(a.motor, idx, float(a.write[1]) if not a.int else int(a.write[1], 0), not a.int)
        print("written (RAM only)")


def cmd_disable(L, a):
    L.disable(mask(*motors_arg(a.m)), clear_fault=a.clear)
    print("disabled")


# ================================================================ main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", required=True, help="COMx (USB-C of the board) or /dev/ttyACMx")
    ap.add_argument("--ids", type=int, nargs=4, default=[1, 2, 1, 2], metavar=("M1", "M2", "M3", "M4"),
                    help="CAN IDs: M1 M2 on CAN1, M3 M4 on CAN2 (default 1 2 1 2)")
    ap.add_argument("--tscale", type=float, default=5.5, help="type-1 torque field range +-Nm (RS05 = 5.5)")
    ap.add_argument("--soft-min", type=float, default=-110.0)
    ap.add_argument("--soft-max", type=float, default=110.0)
    ap.add_argument("--temp-max", type=float, default=75.0)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def with_m(name, default=("1",)):
        s = sub.add_parser(name)
        s.add_argument("-m", nargs="+", default=list(default), help="motors 1..4 or 'all'")
        return s

    sub.add_parser("info")
    sub.add_parser("health")
    sub.add_parser("monitor")
    s = sub.add_parser("setid")
    s.add_argument("--motor", type=int, choices=(1, 2, 3, 4), required=True)
    s.add_argument("--new", type=int, required=True)
    with_m("zero", ("all",))
    with_m("save")
    s = with_m("disable", ("all",)); s.add_argument("--clear", action="store_true")
    s = with_m("home")
    s.add_argument("--dir", type=int, choices=(-1, 1), required=True)
    s.add_argument("--speed", type=float, default=0.5)
    s.add_argument("--cur", type=float, default=1.5)
    s.add_argument("--backoff", type=float, default=0.2)
    s = with_m("pos")
    s.add_argument("--targets", type=float, nargs="+", default=[90, -90, 45, -45])
    s.add_argument("--vlim", type=float, default=1.0)
    s.add_argument("--cur", type=float, default=3.0)
    s.add_argument("--tol", type=float, default=1.0)
    s.add_argument("--dwell", type=float, default=2.0)
    s = with_m("vel-sweep")
    s.add_argument("--rpm", type=int, nargs=3, default=[20, 300, 20])
    s.add_argument("--cur", type=float, default=8.0)
    s.add_argument("--acc", type=float, default=10.0)
    s.add_argument("--settle", type=float, default=1.5)
    s.add_argument("--hold", type=float, default=2.0)
    s.add_argument("--i-understand-rotation", action="store_true")
    s = with_m("torque")
    s.add_argument("--steps", type=float, nargs="+", default=[0.1, 0.2, 0.4, 0.6])
    s.add_argument("--duration", type=float, default=0.5)
    s.add_argument("--rest", type=float, default=1.0)
    s.add_argument("--kd", type=float, default=0.05)
    s.add_argument("--both-dirs", action="store_true")
    s = with_m("mit")
    s.add_argument("--p", type=float, default=None)
    s.add_argument("--v", type=float, default=0.0)
    s.add_argument("--kp", type=float, default=5.0)
    s.add_argument("--kd", type=float, default=0.3)
    s.add_argument("--t", type=float, default=0.0)
    s.add_argument("--duration", type=float, default=10.0)
    s = sub.add_parser("param")
    s.add_argument("--motor", type=int, choices=(1, 2, 3, 4), default=1)
    s.add_argument("--read")
    s.add_argument("--write", nargs=2)
    s.add_argument("--int", action="store_true")

    a = ap.parse_args()
    fn = {"info": cmd_info, "health": cmd_health, "monitor": cmd_monitor, "setid": cmd_setid,
          "zero": cmd_zero, "save": cmd_save, "disable": cmd_disable, "home": cmd_home, "pos": cmd_pos,
          "vel-sweep": cmd_vel_sweep, "torque": cmd_torque, "mit": cmd_mit, "param": cmd_param}[a.cmd]
    with RS05Link(a.port) as L:
        try:
            setup(L, a)
            fn(L, a)
        except KeyboardInterrupt:
            print("\nCtrl+C -> E-STOP")
            L.estop()
            time.sleep(0.1)


if __name__ == "__main__":
    main()
