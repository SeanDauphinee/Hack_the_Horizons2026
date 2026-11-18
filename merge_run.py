#!/usr/bin/env python3
"""Merge one run folder into ONE CSV laid out like Tristan's Simulink Meas_Bus:  <run>/combined.csv

    python3 merge_run.py ~/runs/run_20261111_141500_walk1

One row per IMU sample (~100 Hz = base rate). Vector bus elements are split into numbered columns
(imuAccel -> imuAccel_1..3). Columns, in bus order, then utc:
    imuAccel_1..3      [m/s^2]  specific force, body x fwd, y right, z down (still & flat -> _3 ~ -9.81)
    imuGyro_1..3       [rad/s]  body rates, same axes
    imuValid           1 on every row (each row IS an IMU sample)
    baroAltitude       [m]      ISA pressure altitude (positive UP, absolute, has a weather bias)
    baroValid          1 only on the row where a NEW baro sample arrived
    lteRanges_1..6 [m]  calibrated LTE pseudo-range to tower k (k = lteSourceId = row of lte_towers.csv);
                        = geometric range + c*(receiver clock change since calibration, SAME for all towers)
    lteValid_1..6      1 on the row where a new range for that tower arrived (~1 Hz); 0 if no LTE recorded
    lteSourceId_1..6   tower id 1..6 (lte_towers.csv / lte_towers_ned.csv), 0 = slot not used
    leoRangeRates_1..12, leoValid_1..12, leoSourceId_1..12  no LEO receiver      -> 0 / 0 / 0
    gnssPositionNED_1..3 [m]    North, East, Down from the FIRST GPS fix of the run (origin in README)
    gnssVelocityNED_1..3 [m/s]  from GPS speed + course; Down velocity = 0 (NMEA has no vertical speed)
    gnssValid          1 only on the row where a NEW GPS epoch WITH a fix arrived
    timestamp          [s]      time since logger start (t = 0 at start)
    utc                ISO 8601 UTC of the same instant (Pi clock, disciplined by chrony + GPS PPS)
When a *Valid flag is 0 the value columns hold the last valid value (zero-order hold; 0 before the first).
Booleans are written as 0/1. The raw imu/baro/gnss/lora/events CSVs stay in the folder as backup.
"""
import csv
import math
import os
import re
import sys
from datetime import datetime, timezone

N_LTE, N_LEO = 6, 12


def vec(name, n):
    return [f"{name}_{i}" for i in range(1, n + 1)]


COLUMNS = (vec("imuAccel", 3) + vec("imuGyro", 3) + ["imuValid", "baroAltitude", "baroValid"]
           + vec("lteRanges", N_LTE) + vec("lteValid", N_LTE) + vec("lteSourceId", N_LTE)
           + vec("leoRangeRates", N_LEO) + vec("leoValid", N_LEO) + vec("leoSourceId", N_LEO)
           + vec("gnssPositionNED", 3) + vec("gnssVelocityNED", 3) + ["gnssValid", "timestamp", "utc"])

# ---------------------------------------------------------------- WGS-84 geodetic -> local NED
A, E2 = 6378137.0, 6.69437999014e-3


def ecef(lat, lon, h):
    la, lo = math.radians(lat), math.radians(lon)
    n = A / math.sqrt(1 - E2 * math.sin(la) ** 2)
    return ((n + h) * math.cos(la) * math.cos(lo), (n + h) * math.cos(la) * math.sin(lo),
            (n * (1 - E2) + h) * math.sin(la))


def ned(lat, lon, h, origin):
    lat0, lon0, h0 = origin
    x, y, z = ecef(lat, lon, h)
    x0, y0, z0 = ecef(lat0, lon0, h0)
    dx, dy, dz = x - x0, y - y0, z - z0
    la, lo = math.radians(lat0), math.radians(lon0)
    n = -math.sin(la) * math.cos(lo) * dx - math.sin(la) * math.sin(lo) * dy + math.cos(la) * dz
    e = -math.sin(lo) * dx + math.cos(lo) * dy
    d = -math.cos(la) * math.cos(lo) * dx - math.cos(la) * math.sin(lo) * dy - math.sin(la) * dz
    return n, e, d


# ---------------------------------------------------------------- helpers
def read(path):
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


def start_epoch(run):
    try:
        with open(os.path.join(run, "README.txt")) as f:
            m = re.search(r"Start epoch\s*:\s*([0-9.]+)", f.read())
        return float(m.group(1)) if m else None
    except OSError:
        return None


def add_readme_note(run, text):
    p = os.path.join(run, "README.txt")
    if os.path.exists(p):
        with open(p) as f:
            if text in f.read():
                return
        with open(p, "a") as f:
            f.write(text)


GEOID_OTTAWA_M = -34.0      # geoid undulation near Ottawa: ellipsoidal h = MSL + (-34 m)


def write_tower_ned(run, origin):
    """Tower positions in the SAME NED frame as gnssPositionNED -> <run>/lte_towers_ned.csv"""
    src = os.path.join(os.path.dirname(os.path.abspath(__file__)), "lte_towers.csv")
    if not (origin and os.path.exists(src)):
        return
    with open(src) as f, open(os.path.join(run, "lte_towers_ned.csv"), "w", newline="") as g:
        w = csv.writer(g)
        w.writerow(["source_id", "tower_id", "north_m", "east_m", "down_m", "nid1", "pcis"])
        for t in csv.DictReader(f):
            h = float(t["ground_msl_m"]) + float(t["antenna_agl_m"]) + GEOID_OTTAWA_M
            n, e, d = ned(float(t["lat_deg"]), float(t["lon_deg"]), h, origin)
            w.writerow([t["source_id"], t["tower_id"], f"{n:.1f}", f"{e:.1f}", f"{d:.1f}", t["nid1"], t["pcis"]])


def merge(run):
    imu = read(os.path.join(run, "imu.csv"))
    if not imu:
        print("merge: imu.csv empty, nothing to merge")
        return None
    baro = read(os.path.join(run, "baro.csv"))
    gnss = [g for g in read(os.path.join(run, "gnss.csv"))
            if g["fix_quality"] and int(g["fix_quality"]) > 0 and g["lat_deg"]]
    origin = (float(gnss[0]["lat_deg"]), float(gnss[0]["lon_deg"]), float(gnss[0]["height_m"])) if gnss else None
    t0 = start_epoch(run)

    # LTE pseudo-ranges (from lte_process.py): one slot per tower, slot = source_id - 1
    lte = [r for r in read(os.path.join(run, "lte.csv")) if r.get("source_id") and r.get("pseudorange_m")]
    lte.sort(key=lambda r: float(r["time_s"]))
    li = 0
    lte_rng, lte_id = ["0"] * N_LTE, ["0"] * N_LTE
    nl = 0
    zeros_leo = ["0"] * N_LEO
    bi = gi = 0
    baro_alt, pos, vel = "0", ["0", "0", "0"], ["0", "0", "0"]
    nb = ng = 0
    out_path = os.path.join(run, "combined.csv")
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(COLUMNS)
        for r in imu:
            t = float(r["time_s"])
            b_valid = 0
            while bi < len(baro) and float(baro[bi]["time_s"]) <= t:
                baro_alt, b_valid = baro[bi]["alt_isa_m"], 1
                bi += 1
            nb += b_valid
            g_valid = 0
            while gi < len(gnss) and float(gnss[gi]["time_s"]) <= t:
                g = gnss[gi]
                n, e, d = ned(float(g["lat_deg"]), float(g["lon_deg"]), float(g["height_m"]), origin)
                pos = [f"{n:.3f}", f"{e:.3f}", f"{d:.3f}"]
                s = float(g["speed_mps"] or 0)
                c = math.radians(float(g["course_deg"] or 0))
                vel = [f"{s * math.cos(c):.3f}", f"{s * math.sin(c):.3f}", "0"]
                g_valid = 1
                gi += 1
            ng += g_valid
            lte_val = ["0"] * N_LTE
            seen = {}
            while li < len(lte) and float(lte[li]["time_s"]) <= t:
                r_ = lte[li]
                sid = int(r_["source_id"])
                if 1 <= sid <= N_LTE and float(r_["pss_metric"]) > seen.get(sid, -1):
                    seen[sid] = float(r_["pss_metric"])          # strongest sector/carrier of that tower
                    lte_rng[sid - 1], lte_id[sid - 1], lte_val[sid - 1] = r_["pseudorange_m"], str(sid), "1"
                li += 1
            nl += sum(v == "1" for v in lte_val)
            utc = ""
            if t0 is not None:
                utc = datetime.fromtimestamp(t0 + t, timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"
            w.writerow([r["ax_mps2"], r["ay_mps2"], r["az_mps2"], r["gx_radps"], r["gy_radps"], r["gz_radps"],
                        1, baro_alt, b_valid]
                       + lte_rng + lte_val + lte_id + zeros_leo * 3
                       + pos + vel + [g_valid, r["time_s"], utc])
    write_tower_ned(run, origin)
    if origin:
        add_readme_note(run, f"NED origin    : first GPS fix lat {origin[0]:.8f} lon {origin[1]:.8f} "
                             f"h {origin[2]:.3f} m (WGS-84 ellipsoidal)  -> gnssPositionNED in combined.csv\n")
    dur = float(imu[-1]["time_s"]) - float(imu[0]["time_s"])
    print(f"merge: combined.csv  {len(imu)} rows ({len(imu) / max(dur, 1e-9):.1f} Hz), "
          f"baroValid {nb}, gnssValid {ng}, lteValid {nl}" + ("" if origin else "  (no GPS fix: NED left at 0)"))
    return out_path


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    merge(os.path.expanduser(sys.argv[1]))
