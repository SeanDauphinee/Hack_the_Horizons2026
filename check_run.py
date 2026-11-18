#!/usr/bin/env python3
"""Check a recorded run before sending it to the team (run on the laptop or the Pi).

    python3 check_run.py path/to/run_YYYYMMDD_HHMMSS

Prints rates, gaps, static-period averages and GNSS fix statistics, and flags problems.
Saves quicklook.png in the run folder if matplotlib is installed.
"""
import csv
import math
import os
import sys


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return rows


def col(rows, name):
    out = []
    for r in rows:
        v = r.get(name, "")
        out.append(float(v) if v not in ("", None) else math.nan)
    return out


def flag(ok, msg):
    print(("  OK    " if ok else "  CHECK ") + msg)
    return ok


def main(run):
    imu, baro, gnss = (load(os.path.join(run, f)) for f in ("imu.csv", "baro.csv", "gnss.csv"))
    events = load(os.path.join(run, "events.csv"))
    print(f"Run: {run}")
    all_ok = True

    t = col(imu, "time_s")
    dur = t[-1] - t[0] if len(t) > 1 else 0
    dts = [b - a for a, b in zip(t, t[1:])]
    rate = (len(t) - 1) / dur if dur else 0
    big = sum(1 for d in dts if d > 0.05)
    print(f"\nIMU: {len(t)} rows, {dur:.1f} s, {rate:.1f} Hz, max gap {max(dts) * 1000:.0f} ms")
    all_ok &= flag(rate > 90, "rate above 90 Hz")
    all_ok &= flag(big == 0, f"gaps > 50 ms: {big}")
    n0 = min(len(imu), 1000)                       # first ~10 s should be static
    ax, ay, az = (sum(col(imu[:n0], c)) / n0 for c in ("ax_mps2", "ay_mps2", "az_mps2"))
    g = math.sqrt(ax * ax + ay * ay + az * az)
    gb = [sum(col(imu[:n0], c)) / n0 for c in ("gx_radps", "gy_radps", "gz_radps")]
    print(f"  first 10 s mean accel ({ax:.2f}, {ay:.2f}, {az:.2f}) |g| = {g:.2f} m/s2")
    print(f"  first 10 s mean gyro (deg/s) {[round(math.degrees(v), 3) for v in gb]}")
    all_ok &= flag(abs(g - 9.81) < 0.5, "gravity magnitude ~9.81 (rig still at start)")
    all_ok &= flag(az < -8.0, "az ~ -9.81 when flat (axis mapping correct)")

    tb = col(baro, "time_s")
    p = col(baro, "pressure_pa")
    rb = (len(tb) - 1) / (tb[-1] - tb[0]) if len(tb) > 1 else 0
    print(f"\nBARO: {len(tb)} rows, {rb:.1f} Hz, pressure {min(p):.0f}..{max(p):.0f} Pa")
    all_ok &= flag(rb > 10, "rate above 10 Hz")
    all_ok &= flag(95000 < sum(p) / len(p) < 105000, "pressure in normal range")

    q = col(gnss, "fix_quality")
    n_fix = sum(1 for v in q if v and v > 0)
    sats = [v for v, qq in zip(col(gnss, "num_sats"), q) if qq and qq > 0]
    print(f"\nGNSS: {len(gnss)} epochs, {n_fix} with fix ({100 * n_fix / max(len(gnss), 1):.0f}%), "
          f"median sats {sorted(sats)[len(sats) // 2] if sats else 0}")
    all_ok &= flag(n_fix > 0, "at least one GPS fix")

    lpath = os.path.join(run, "lora.csv")
    if os.path.exists(lpath):
        lora = load(lpath)
        rs = [v for v in col(lora, "rssi_dbm") if v == v]
        print(f"\nLORA (optional): {len(lora)} packets from {len({r['from_id'] for r in lora})} nodes"
              + (f", RSSI {min(rs):.0f}..{max(rs):.0f} dBm" if rs else ""))
        print("  [ INFO ] no packets: fine if no LoRa node was plugged in or nobody transmitted" if not lora else
              "  [ OK ] lora.csv has data")

    lte_path = os.path.join(run, "lte.csv")
    if os.path.exists(lte_path):
        lte = load(lte_path)
        srcs = {}
        for r in lte:
            srcs.setdefault(r["source"], r)
        used = [r for r in lte if r.get("pseudorange_m")]
        print(f"\nLTE: {len(lte)} arrivals from {len(srcs)} cells, {len(used)} usable ranges "
              f"(cells with a tower: {sorted({r['source_id'] for r in used})})")
        print("  [ OK ] lteRanges filled in combined.csv" if used else
              "  [ INFO ] no calibrated ranges: tower not identified (fill nid1 in lte_towers.csv) or no GPS fix")

    print(f"\nEVENTS ({len(events)}): " + ", ".join(f"{float(e['time_s']):.0f}s {e['event']}" for e in events))
    print("\nRESULT: " + ("ready to send" if all_ok else "review the CHECK lines before sending"))

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(2, 2, figsize=(12, 7))
        for c in ("ax_mps2", "ay_mps2", "az_mps2"):
            axs[0, 0].plot(t, col(imu, c), lw=0.5, label=c)
        axs[0, 0].set_title("Accelerometer (m/s2)"); axs[0, 0].legend(fontsize=7)
        for c in ("gx_radps", "gy_radps", "gz_radps"):
            axs[0, 1].plot(t, col(imu, c), lw=0.5, label=c)
        axs[0, 1].set_title("Gyro (rad/s)"); axs[0, 1].legend(fontsize=7)
        axs[1, 0].plot(tb, col(baro, "alt_isa_m"), lw=0.5)
        axs[1, 0].set_title("Baro altitude (m, ISA)")
        axs[1, 1].plot(col(gnss, "lon_deg"), col(gnss, "lat_deg"), ".", ms=2)
        axs[1, 1].set_title("GNSS track (lon, lat)")
        for e in events:
            for a in (axs[0, 0], axs[0, 1], axs[1, 0]):
                a.axvline(float(e["time_s"]), color="gray", lw=0.5, ls=":")
        fig.tight_layout()
        fig.savefig(os.path.join(run, "quicklook.png"), dpi=100)
        print(f"Saved {os.path.join(run, 'quicklook.png')}")
    except ImportError:
        print("(install matplotlib for quicklook.png)")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1])
