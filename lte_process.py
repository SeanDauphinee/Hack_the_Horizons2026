#!/usr/bin/env python3
"""Turn the LTE bursts of a run into per-cell pseudo-ranges:  <run>/lte.csv

    python3 lte_process.py ~/runs/run_20261111_141500_walk1 [--start LAT,LON,H]

For every burst: find every LTE sync arrival (PSS), its sub-sample arrival time on the LimeSDR clock,
the frequency offset (-> receiver clock error in ppm), and the cell ID when the SSS can be read.
Arrivals are tracked from burst to burst as "sources" (same carrier, same N_ID2, same 5 ms phase).

Pseudo-range model (per source i):   rho_i = c * (arrival time mod 5 ms)   [unwrapped]
                                           = range_i + c*(tower_offset_i + receiver_clock)
Calibration: at the first GPS fixes (or at --start if no GPS) the constant tower_offset_i is removed, so
    pseudorange_m = range_i + c*(receiver clock change since calibration)      <- common to all sources
Sources map to towers through lte_towers.csv (columns nid1 / pcis) -> source_id 1..6 = lteSourceId.
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lte_dsp as L  # noqa: E402
import merge_run as M  # noqa: E402

C = 299792458.0
HF = 0.005                      # 5 ms PSS period
TRACK_GATE_S = 15e-6            # a source may move at most 15 us (4.5 km) between bursts
N_CAL = 10                      # epochs used for calibration


class _Bursts:
    """Lazy list of (row, iq) - reads one burst at a time from lte_iq.bin (memory-mapped),
    so long runs do not fill the Pi's RAM."""

    def __init__(self, run):
        self.idx = list(csv.DictReader(open(os.path.join(run, "lte_bursts.csv"))))
        self.raw = np.memmap(os.path.join(run, "lte_iq.bin"), dtype=np.int16, mode="r")
        self.idx = [r for r in self.idx
                    if int(r["byte_offset"]) // 2 + 2 * int(r["n_samples"]) <= len(self.raw)]

    def __len__(self):
        return len(self.idx)

    def __bool__(self):
        return bool(self.idx)

    def _get(self, r):
        n, off = int(r["n_samples"]), int(r["byte_offset"]) // 2
        q = np.asarray(self.raw[off:off + 2 * n], dtype=np.float32)
        return (q[0::2] + 1j * q[1::2]).astype(np.complex64) * np.float32(float(r["scale"]) / 32767.0)

    def __iter__(self):
        for r in self.idx:
            yield r, self._get(r)

    def __getitem__(self, i):
        return self.idx[i], self._get(self.idx[i])


def load_bursts(run):
    return _Bursts(run)


MANUAL_LINKS = {}           # PCI -> tower_id, from --link 308=C5025,148=C0313


def tower_maps():
    p = os.path.join(os.path.dirname(os.path.abspath(__file__)), "lte_towers.csv")
    by_nid1, by_pci, towers = {}, {}, {}
    with open(p) as f:
        for t in csv.DictReader(f):
            sid = int(t["source_id"])
            towers[sid] = t
            for v in str(t.get("nid1", "")).split(";"):
                if v.strip():
                    by_nid1[int(v)] = sid
            for v in str(t.get("pcis", "")).split(";"):
                if v.strip():
                    by_pci[int(v)] = sid
    by_tid = {t["tower_id"].upper(): sid for sid, t in towers.items()}
    for pci, tid in MANUAL_LINKS.items():
        if tid.upper() not in by_tid:
            raise SystemExit(f"--link: tower '{tid}' is not in lte_towers.csv (ids: {', '.join(sorted(by_tid))})")
        by_pci[pci] = by_tid[tid.upper()]
    return by_nid1, by_pci, towers


def _gps_track(run):
    g = [g for g in M.read(os.path.join(run, "gnss.csv"))
         if g["fix_quality"] and int(g["fix_quality"]) > 0 and g["lat_deg"]]
    t = np.array([float(x["time_s"]) for x in g])
    p = np.array([M.ecef(float(x["lat_deg"]), float(x["lon_deg"]), float(x["height_m"])) for x in g]) if g else None
    return t, p


def geo_associate(sources, run, towers, verbose=True, min_n=15, min_move_m=40.0, max_rms=40.0):
    """Assign towers to unidentified sources by testing every assignment against the GPS track."""
    import itertools
    tg, pg = _gps_track(run)
    if pg is None or len(tg) < min_n:
        return
    if np.linalg.norm(pg.max(0) - pg.min(0)) < min_move_m:
        if verbose:
            print("   (geometry ID skipped: walk at least ~40 m with GPS to identify towers automatically)")
        return
    T = {}
    for sid, t in towers.items():
        h = float(t["ground_msl_m"]) + float(t["antenna_agl_m"]) + M.GEOID_OTTAWA_M
        T[sid] = np.array(M.ecef(float(t["lat_deg"]), float(t["lon_deg"]), h))
    # per source: epoch (rounded second) -> raw pseudo-range
    cand = [s for s in sources if s["n"] >= min_n]
    if len(cand) < 2:
        return
    ser = {}
    for s in cand:
        d = {}
        for m in s["rows"]:
            i = int(np.argmin(np.abs(tg - m["time_s"])))
            if abs(tg[i] - m["time_s"]) < 0.6:
                d[int(round(m["time_s"]))] = (C * m["unwrapped"], pg[i])
        ser[s["key"]] = d
    known = [s for s in cand if s["source_id"] is not None]
    unknown = [s for s in cand if s["source_id"] is None]
    if not unknown:
        return
    unknown = sorted(unknown, key=lambda s: -s["n"])[:4]
    sids = sorted(T)
    best = []
    for assign in itertools.product(sids, repeat=len(unknown)):
        A = {s["key"]: s["source_id"] for s in known}
        A.update({s["key"]: a for s, a in zip(unknown, assign)})
        keys = list(A)
        ep = set.intersection(*[set(ser[k]) for k in keys])
        if len(ep) < min_n:
            continue
        ep = sorted(ep)
        R = np.array([[ser[k][e][0] - np.linalg.norm(ser[k][e][1] - T[A[k]]) for e in ep] for k in keys])
        R = R - R.mean(1, keepdims=True)          # each tower's constant offset
        R = R - R.mean(0, keepdims=True)          # receiver clock (common to all)
        best.append((float(np.sqrt(np.mean(R ** 2))), assign))
    if not best:
        return
    best.sort()
    rms = best[0][0]
    near = [b for b in best if b[0] <= max(rms * 1.3, rms + 3.0)]
    decided = {}
    for j, s in enumerate(unknown):
        choices = {b[1][j] for b in near}
        if len(choices) == 1 and rms < max_rms:
            decided[s["key"]] = choices.pop()
    if verbose:
        print(f"   geometry ID: best fit {rms:.1f} m rms; identified {len(decided)}/{len(unknown)} cells"
              + ("" if len(decided) == len(unknown) else " (walk a longer, non-straight route to identify the rest)"))
    for s in unknown:
        if s["key"] in decided:
            s["source_id"], s["id_method"] = decided[s["key"]], "geometry"


def shape_match(sources, run, towers, verbose=True, min_n=60, min_corr=0.6, min_gap=0.3, min_change_m=300.0):
    tg, pg = _gps_track(run)
    if pg is None or len(tg) < min_n:
        return
    T = {}
    for sid, t in towers.items():
        h = float(t["ground_msl_m"]) + float(t["antenna_agl_m"]) + M.GEOID_OTTAWA_M
        T[sid] = np.array(M.ecef(float(t["lat_deg"]), float(t["lon_deg"]), h))
    for s in sources:
        if s["n"] < min_n:
            continue
        rows = [m for m in s["rows"] if abs(tg[int(np.argmin(np.abs(tg - m["time_s"])))] - m["time_s"]) < 0.6]
        if len(rows) < min_n:
            continue
        t = np.array([m["time_s"] for m in rows])
        y = np.array([C * m["unwrapped"] for m in rows])
        idx = [int(np.argmin(np.abs(tg - v))) for v in t]
        V = np.vander((t - t.mean()) / (t.std() + 1e-9), 3)
        ly = y - V @ np.linalg.lstsq(V, y, rcond=None)[0]
        res = []
        for sid, tp in T.items():
            r = np.linalg.norm(pg[idx] - tp, axis=1)
            g = r - V @ np.linalg.lstsq(V, r, rcond=None)[0]
            cc = float(np.corrcoef(ly, g)[0, 1]) if g.std() > 1 else 0.0
            res.append((cc, sid, float(np.ptp(r))))
        res.sort(reverse=True)
        best, second = res[0], res[1]
        txt = ", ".join(f"{towers[sid]['tower_id']} {cc:+.2f}" for cc, sid, _ in res[:3])
        ok = best[0] >= min_corr and best[0] - second[0] >= min_gap and best[2] >= min_change_m
        s["shape"] = (towers[best[1]]["tower_id"], best[0])
        if verbose:
            pci = s.get("pci") if s.get("pci") is not None else "?"
            verdict = ("LINKED" if ok and s["source_id"] is None else
                       "agrees" if ok else
                       f"suggests {towers[best[1]]['tower_id']} (not sure enough: need corr>={min_corr}, "
                       f"gap>={min_gap}, distance change>={min_change_m:.0f} m, got {best[2]:.0f} m)")
            print(f"   shape match PCI {pci}: {txt} -> {verdict}")
        if ok and s["source_id"] is None:
            s["source_id"], s["id_method"] = best[1], "shape"


def process(run, start=None, verbose=True):
    bursts = load_bursts(run)
    if not bursts:
        print("lte_process: no bursts")
        return None
    t_ref = None
    meas = []                               # one dict per detected arrival
    # pass 1 (slow, full frequency search) only on ~24 bursts spread over the run: enough to fit the clock
    step = max(1, len(bursts) // 40)
    for bi, r in enumerate(bursts.idx):
        f_mhz = float(r["freq_mhz"])
        hw = float(r["hw_time_ns"]) * 1e-9 if r["hw_time_ns"] else None
        t_pi = float(r["time_s"])
        if t_ref is None:
            t_ref = hw if hw is not None else t_pi
        if bi % step:
            continue
        x = bursts[bi][1]
        peaks = L.find_pss_peaks(x)
        xc = np.asarray(x, complex)
        for nid2, kf, metric, cfo0 in peaks:
            k = int(round(kf)) % L.HALF_FRAME
            n = np.arange(len(xc))
            starts = [k + p * L.HALF_FRAME for p in range(len(xc) // L.HALF_FRAME)
                      if k + p * L.HALF_FRAME + L.NFFT <= len(xc)]
            fine = L._cfo_from_pss(xc * np.exp(-2j * np.pi * cfo0 * n / L.FS), starts, nid2)
            cfo = cfo0 + (fine if abs(fine) < 2000 else 0.0)
            nid1, q = None, 0.0
            meas.append(dict(burst=int(r["burst"]), time_s=t_pi + kf / L.FS, freq=f_mhz, nid2=nid2,
                             hw=(hw if hw is not None else t_pi) + kf / L.FS, metric=metric, cfo=cfo,
                             nid1=nid1, sss_q=q, has_hw=hw is not None))
    if not meas:
        print("lte_process: no LTE sync arrivals found in any burst")
        return None

    # receiver clock error from the frequency offset of the strongest arrivals (towers are ~0.05 ppm)
    ppm_burst = {}
    for b in sorted({m["burst"] for m in meas}):
        ms = sorted([m for m in meas if m["burst"] == b], key=lambda m: -m["metric"])[:3]
        ppm_burst[b] = float(np.median([-m["cfo"] / (m["freq"] * 1e6) * 1e6 for m in ms]))
    bs = sorted(ppm_burst)
    vals = np.array([ppm_burst[b] for b in bs])
    hw_b = {}
    for m in meas:
        hw_b.setdefault(m["burst"], m["hw"])
    tb = np.array([hw_b[b] - t_ref for b in bs])
    # ONE smooth clock model for the whole run (same receiver clock for every carrier):
    # eps(t) = a + b*t (robust fit), integrated -> time correction a*t + b*t^2/2
    good = np.abs(vals - np.median(vals)) < 0.5
    if good.sum() >= 3 and np.ptp(tb[good]) > 5:
        b_, a_ = np.polyfit(tb[good], vals[good], 1)
    else:
        a_, b_ = float(np.median(vals)), 0.0

    def ppm_at(t):
        return a_ + b_ * t

    # pass 2: re-detect every burst at the frequency offset PREDICTED by the clock model
    # (cfo = -eps*f_carrier). PSS timing is coupled to frequency error, so this removes timing jitter.
    meas2 = []
    for bi, (r, x) in enumerate(bursts):
        if verbose and bi % 50 == 0:
            print(f"   LTE processing {bi}/{len(bursts)} bursts", flush=True)
        f_mhz = float(r["freq_mhz"])
        hw = float(r["hw_time_ns"]) * 1e-9 if r["hw_time_ns"] else None
        t_pi = float(r["time_s"])
        hw0 = hw if hw is not None else t_pi
        cfo_pred = -ppm_at(hw0 - t_ref) * 1e-6 * f_mhz * 1e6
        xc = np.asarray(x, complex)
        pk = L.find_pss_peaks(xc, cfo=cfo_pred)
        if pk:                                   # refine the offset on this burst's strongest cell
            nid2s, kfs = pk[0][0], int(round(pk[0][1])) % L.HALF_FRAME
            nn = np.arange(len(xc))
            st = [kfs + p * L.HALF_FRAME for p in range(len(xc) // L.HALF_FRAME)
                  if kfs + p * L.HALF_FRAME + L.NFFT <= len(xc)]
            fine = L._cfo_from_pss(xc * np.exp(-2j * np.pi * cfo_pred * nn / L.FS), st, nid2s)
            if abs(fine) < 1500:
                pk = L.find_pss_peaks(xc, cfo=cfo_pred + fine)
        for nid2, kf, metric, cfo in pk:
            nid1, q = L.sss_decode_at(xc, nid2, int(round(kf)) % L.HALF_FRAME, cfo)
            meas2.append(dict(burst=int(r["burst"]), time_s=t_pi + kf / L.FS, freq=f_mhz, nid2=nid2,
                              hw=hw0 + kf / L.FS, metric=metric, cfo=cfo, nid1=nid1, sss_q=q,
                              has_hw=hw is not None))
    if meas2:
        meas = meas2

    # 5 ms phase of each arrival on the RAW LimeSDR sample clock.
    # Real data (10 Nov): the frequency offset (~-4.7 kHz = 5.4 ppm) is mostly a SYNTHESIZER tuning offset,
    # NOT a sample-clock error: the tower timing drifted < 0.1 us/s on the raw clock. So the timing is NOT
    # corrected with the frequency offset; any small receiver clock drift is common to all towers and is
    # estimated by the fusion filter (clock bias + drift states).
    for m in meas:
        t = m["hw"] - t_ref
        m["ppm"] = ppm_at(t)                     # frequency offset in ppm (info only)
        m["t_corr"] = m["hw"]
        m["phase"] = (m["t_corr"] - t_ref) % HF

    # track sources: same carrier + N_ID2, phase continuous
    sources = []                           # dicts: key, freq, nid2, last_phase, unwrap, rows
    for b in sorted({m["burst"] for m in meas}):
        for m in sorted([m for m in meas if m["burst"] == b], key=lambda m: -m["metric"]):
            best, bd, dbest = None, TRACK_GATE_S, 0.0
            for s in sources:
                if s["freq"] != m["freq"] or s["nid2"] != m["nid2"] or s["last_burst"] == b:
                    continue
                d = (m["phase"] - s["last_phase"] + HF / 2) % HF - HF / 2
                if abs(d) < bd:
                    best, bd, dbest = s, abs(d), d
            d = dbest
            if best is None:
                best = dict(key=f"{m['freq']:.1f}/{m['nid2']}/{len([s for s in sources if s['freq'] == m['freq'] and s['nid2'] == m['nid2']])}",
                            freq=m["freq"], nid2=m["nid2"], last_phase=m["phase"], acc=m["phase"], rows=[],
                            last_burst=None)
                sources.append(best)
                d = 0.0
            best["acc"] = best["acc"] + d if best["rows"] else m["phase"]
            best["last_phase"], best["last_burst"] = m["phase"], b
            m["source"] = best["key"]
            m["unwrapped"] = best["acc"]
            best["rows"].append(m)

    # drop 'ghost' sources: same carrier, same timing as a stronger source (PSS cross-correlation)
    def _phase_map(s):
        return {m["burst"]: m["phase"] for m in s["rows"]}
    for s in sources:
        s["n"] = len(s["rows"])
        s["metric"] = float(np.median([m["metric"] for m in s["rows"]]))
    keep = []
    for s in sorted(sources, key=lambda s: -s["metric"]):
        ghost = False
        for k in keep:
            if k["freq"] != s["freq"]:
                continue
            a, b = _phase_map(k), _phase_map(s)
            common = set(a) & set(b)
            if common and np.median([abs((a[c] - b[c] + HF / 2) % HF - HF / 2) for c in common]) < 2e-6:
                ghost = True
                break
        if not ghost and s["n"] >= 3:
            keep.append(s)
    dropped = {s["key"] for s in sources} - {s["key"] for s in keep}
    meas = [m for m in meas if m["source"] not in dropped]
    sources = keep

    # cell ID per source by vote, then tower
    by_nid1, by_pci, towers = tower_maps()
    # one source per cell: other tracks of the SAME PCI are reflections / side peaks -> drop them
    for s in sources:
        ids = [m["nid1"] for m in s["rows"] if m["nid1"] is not None]
        s["_pci"] = 3 * max(set(ids), key=ids.count) + s["nid2"] if ids else None
    keep = []
    for s in sorted(sources, key=lambda s: -s["n"]):
        if s["_pci"] is not None and any(k["_pci"] == s["_pci"] and k["freq"] == s["freq"] for k in keep):
            continue
        keep.append(s)
    dropped = {s["key"] for s in sources} - {s["key"] for s in keep}
    meas = [m for m in meas if m["source"] not in dropped]
    sources = keep
    for s in sources:
        ids = [m["nid1"] for m in s["rows"] if m["nid1"] is not None]
        s["nid1"] = max(set(ids), key=ids.count) if ids else None
        s["pci"] = 3 * s["nid1"] + s["nid2"] if s["nid1"] is not None else None
        sid = None
        if s["pci"] is not None:
            sid = by_pci.get(s["pci"], by_nid1.get(s["nid1"]))
        s["source_id"] = sid
        s["id_method"] = ("manual" if s["pci"] in MANUAL_LINKS else "pci") if sid else ""
        s["n"] = len(s["rows"])

    # per-cell SHAPE match (works with ONE cell): remove a smooth clock (quadratic) from the LTE range AND
    # from each tower's GPS distance, then correlate. Real tower = clearly positive and clearly the best.
    shape_match(sources, run, towers, verbose)

    # tower for sources whose ID could not be read: GEOMETRY. While you walk, each range must change
    # exactly like the distance to its tower (after removing a constant and the clock common to all).
    geo_associate(sources, run, towers, verbose)

    # calibration (remove each tower's unknown transmit offset)
    cal_pts = []
    gnss = [g for g in M.read(os.path.join(run, "gnss.csv"))
            if g["fix_quality"] and int(g["fix_quality"]) > 0 and g["lat_deg"]]
    origin = (float(gnss[0]["lat_deg"]), float(gnss[0]["lon_deg"]), float(gnss[0]["height_m"])) if gnss else None
    if gnss:
        cal_pts = [(float(g["time_s"]), (float(g["lat_deg"]), float(g["lon_deg"]), float(g["height_m"])))
                   for g in gnss]
    elif start:
        origin = start
        cal_pts = [(-1e9, start), (1e9, start)]          # static at the start point
    tower_ecef = {}
    for sid, t in towers.items():
        h = float(t["ground_msl_m"]) + float(t["antenna_agl_m"]) + M.GEOID_OTTAWA_M
        tower_ecef[sid] = np.array(M.ecef(float(t["lat_deg"]), float(t["lon_deg"]), h))

    def pos_at(ts):
        if not cal_pts:
            return None
        tt = [c[0] for c in cal_pts]
        i = int(np.argmin(np.abs(np.array(tt) - ts)))
        if abs(tt[i] - ts) > 1.5 and len(cal_pts) > 2:
            return None
        return np.array(M.ecef(*cal_pts[i][1]))

    for s in sources:
        s["offset"] = None
        if s["source_id"] is None or not cal_pts:
            continue
        diffs = []
        for m in s["rows"]:
            p = pos_at(m["time_s"])
            if p is None:
                continue
            diffs.append(C * m["unwrapped"] - np.linalg.norm(p - tower_ecef[s["source_id"]]))
            if len(diffs) >= N_CAL:
                break
        if diffs:
            s["offset"] = float(np.median(diffs))

    out = os.path.join(run, "lte.csv")
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time_s", "burst", "freq_mhz", "source", "nid2", "pci", "source_id", "toa_hw_s",
                    "phase_us", "pseudorange_raw_m", "pseudorange_m", "pss_metric", "sss_q", "cfo_hz", "rx_ppm"])
        for m in sorted(meas, key=lambda m: m["time_s"]):
            s = next(s for s in sources if s["key"] == m["source"])
            raw = C * m["unwrapped"]
            cal = "" if s["offset"] is None else f"{raw - s['offset']:.2f}"
            w.writerow([f"{m['time_s']:.4f}", m["burst"], m["freq"], m["source"], m["nid2"],
                        "" if s["pci"] is None else s["pci"], "" if s["source_id"] is None else s["source_id"],
                        f"{m['hw']:.9f}", f"{m['phase'] * 1e6:.3f}", f"{raw:.2f}", cal,
                        f"{m['metric']:.3f}", f"{m['sss_q']:.2f}", f"{m['cfo']:.0f}", f"{m['ppm']:.3f}"])
    if verbose:
        print(f"lte_process: {len(bursts)} bursts, {len(meas)} arrivals, {len(sources)} sources"
              + ("" if all(m["has_hw"] for m in meas) else "  WARNING: no hardware timestamps, ranges unreliable"))
        for s in sorted(sources, key=lambda s: -s["n"]):
            print(f"   {s['key']:12s} seen {s['n']:4d}x  PCI {s['pci'] if s['pci'] is not None else '?':>4}  "
                  f"tower {s['source_id'] if s['source_id'] else '-'} {('(' + s['id_method'] + ')') if s['source_id'] else '':10s} "
                  f"{'calibrated' if s['offset'] is not None else 'not calibrated'}")
        fo = np.median([m["cfo"] for m in meas])
        print(f"   radio tuned {fo / 1e3:+.1f} kHz from the carrier centre (handled) -> {out}")
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run")
    ap.add_argument("--start", help="known start position lat,lon,h (used if the run has no GPS fix)")
    ap.add_argument("--link", help="link cells to towers by hand, e.g.  308=C5025,148=C0313  (tower ids from lte_towers.csv)")
    a = ap.parse_args()
    if a.link:
        for item in a.link.split(","):
            pci, tid = item.split("=")
            MANUAL_LINKS[int(pci)] = tid.strip()
    st = tuple(float(v) for v in a.start.split(",")) if a.start else None
    process(os.path.expanduser(a.run), st)
