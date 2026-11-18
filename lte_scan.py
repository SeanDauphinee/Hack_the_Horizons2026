#!/usr/bin/env python3
"""LTE Band 5 scanner for the LimeSDR Mini 2.0 (RECEIVE ONLY: this script never opens a TX stream).

Finds LTE carriers in 869-894 MHz and lists every cell it can decode the ID of:
    PCI (Physical Cell ID 0..503), its group N_ID1 (= PCI // 3, shared by the 3 sectors of a site),
    PSS strength (dB above floor), SSS quality (0..1, >0.4 = trusted), frequency offset.

    python3 lte_scan.py                         # full Band 5 scan (~2-4 min on a Pi 4)
    python3 lte_scan.py --label near_C5025      # tag the result file with where you stood
    python3 lte_scan.py --freq 881.5            # only this carrier (fast, for repeated checks)
    python3 lte_scan.py --gain 30               # lower gain if it says CLIPPING (close to a tower)
    python3 lte_scan.py --sim                   # no hardware: synthetic towers

Results: printed table + ~/runs/lte_scans/lte_scan_<time>_<label>.csv
"""
import argparse
import csv
import os
import sys
import time
from datetime import datetime

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lte_dsp as L  # noqa: E402

FS_HW = 3.84e6                     # LimeSDR sample rate; decimated x2 -> 1.92 MS/s
B5_LO, B5_HI = 869.0, 894.0        # Band 5 downlink, MHz


def earfcn(f_mhz):
    return int(round(2400 + (f_mhz - 869.0) / 0.1))


# --------------------------------------------------------------------------- radio front-ends
class LimeRX:
    """SoapySDR receive-only wrapper. Works with legacy LimeSuite (SoapyLMS7) or LimeSuiteNG."""

    def __init__(self, gain):
        import SoapySDR
        from SoapySDR import SOAPY_SDR_RX, SOAPY_SDR_CF32
        self.S, self.RX = SoapySDR, SOAPY_SDR_RX
        def as_dict(kw):                        # SoapySDRKwargs is not a real dict
            return {str(k): str(kw[k]) for k in kw.keys()}

        devs = []
        for query in ("driver=lime", "driver=limesuiteng", ""):   # ask Lime drivers first (no audio probing)
            try:
                devs = [as_dict(d) for d in SoapySDR.Device.enumerate(query)]
            except Exception:
                devs = []
            devs = [d for d in devs if "lime" in (d.get("driver", "") + d.get("label", "") +
                                                  d.get("name", "")).lower()]
            if devs:
                break
        if not devs:
            raise RuntimeError("no LimeSDR found by SoapySDR. Run:  SoapySDRUtil --find")
        d0 = devs[0]
        args = "driver=lime"
        if False:
            args["serial"] = d0["serial"]
        self.label = d0.get("label", d0.get("name", "LimeSDR"))
        self.dev = SoapySDR.Device(args)
        d = self.dev
        d.setSampleRate(SOAPY_SDR_RX, 0, FS_HW)
        try:
            d.setBandwidth(SOAPY_SDR_RX, 0, 5e6)
        except Exception:
            pass
        ants = list(d.listAntennas(SOAPY_SDR_RX, 0))
        for a in ("LNAW", "LNAL"):              # LNAW = wide-band input, right for 880 MHz
            if a in ants:
                d.setAntenna(SOAPY_SDR_RX, 0, a)
                break
        self.antenna = d.getAntenna(SOAPY_SDR_RX, 0)
        d.setGain(SOAPY_SDR_RX, 0, gain)
        self.st = d.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, [0])
        d.activateStream(self.st)
        self.buf = np.empty(65536, np.complex64)
        self._read(int(0.2 * FS_HW))            # warm-up: throw away 200 ms after start-up

    def walk_in(self, f_mhz, start=869.7, step=0.1):
        """Approach f_mhz in small steps from the bottom of the band, exactly like the full scan does.
        (The direct jump to a frequency gave a ~5-10 kHz offset and undecodable IDs on our board.)"""
        f = start
        while f < f_mhz - 1e-6:
            self.dev.setFrequency(self.RX, 0, f * 1e6)
            self._read(int(0.005 * FS_HW))
            f = round(f + step, 3)

    def capture(self, f_mhz, n):
        """Tune, drop 5 ms of settling samples, return n samples at FS_HW and the hardware time (ns)."""
        d = self.dev
        d.setFrequency(self.RX, 0, f_mhz * 1e6)
        self._read(int(0.02 * FS_HW))           # 20 ms settling after a retune
        return self._read(n)

    def _read(self, n):
        out = np.empty(n, np.complex64)
        got, t_ns = 0, None
        tries = 0
        while got < n:
            k = min(len(self.buf), n - got)
            sr = self.dev.readStream(self.st, [self.buf], k, timeoutUs=500000)
            if sr.ret > 0:
                if t_ns is None:
                    t_ns = sr.timeNs
                out[got:got + sr.ret] = self.buf[:sr.ret]
                got += sr.ret
            else:
                tries += 1
                if tries > 20:
                    raise RuntimeError(f"readStream error {sr.ret}")
        return out, t_ns

    def temp_c(self):
        try:
            return float(self.dev.readSensor("lms7_temp"))
        except Exception:
            return float("nan")

    def close(self):
        try:
            self.dev.deactivateStream(self.st)
            self.dev.closeStream(self.st)
        except Exception:
            pass


class SimRX:
    """Fake band: two carriers with 3 cells (two sectors of one site share N_ID1)."""
    label, antenna = "SIMULATED", "sim"

    def temp_c(self):
        return 40.0
    CARRIERS = {881.5: [(246, 1234, 3000, 1.0), (247, 5000, 3000, 0.5)], 891.0: [(400, 333, 1500, 0.6)]}

    def __init__(self, gain):
        self.rng = np.random.default_rng(7)

    def capture(self, f_mhz, n):
        n2 = n // 2
        x = (self.rng.standard_normal(n2) + 1j * self.rng.standard_normal(n2)) * 0.05
        for fc, cells in self.CARRIERS.items():
            if abs(f_mhz - fc) < 0.05:
                x = x + sum(L.synth_cell(n2, p, d, cfo, a * 0.3, self.rng) for p, d, cfo, a in cells)
            elif abs(f_mhz - fc) < 4.5:                       # inside a 10 MHz carrier: power, no PSS
                x = x + (self.rng.standard_normal(n2) + 1j * self.rng.standard_normal(n2)) * 0.2
        up = np.repeat(x, 2)                                  # crude 3.84 MS/s
        return up.astype(np.complex64), None

    def close(self):
        pass


# --------------------------------------------------------------------------- helpers
def power_dbfs(x):
    return 10 * np.log10(np.mean(np.abs(x) ** 2) + 1e-20)


def quick_pss_db(x192):
    best = 0.0
    for nid2 in range(3):
        m, _ = L.pss_search(x192, nid2)
        best = max(best, 10 * np.log10(m.max() / (np.median(m) + 1e-30)))
    return best


def load_towers():
    p = os.path.join(os.path.dirname(os.path.abspath(__file__)), "lte_towers.csv")
    out = {}
    if os.path.exists(p):
        with open(p) as f:
            for t in csv.DictReader(f):
                if t.get("nid1"):
                    for v in str(t["nid1"]).split(";"):
                        if v.strip():
                            out[int(v)] = f"{t['source_id']}:{t['tower_id']}"
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--freq", type=float, help="check one carrier (MHz) instead of scanning")
    ap.add_argument("--start", type=float, default=B5_LO + 0.7)
    ap.add_argument("--stop", type=float, default=B5_HI - 0.7)
    ap.add_argument("--gain", type=float, default=40)
    ap.add_argument("--ms", type=float, default=80, help="capture length per carrier for ID decoding")
    ap.add_argument("--label", default="")
    ap.add_argument("--out", default=os.path.expanduser("~/runs/lte_scans"))
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--no-walk", action="store_true", help="--freq mode: jump directly (old behaviour)")
    ap.add_argument("--debug", action="store_true", help="--freq mode: show all candidates and save raw samples (.npy)")
    a = ap.parse_args()

    try:
        rx = SimRX(a.gain) if a.sim else LimeRX(a.gain)
    except ImportError:
        print("SoapySDR Python module missing. Run:  bash setup_sdr.sh")
        sys.exit(1)
    except Exception as e:
        print(f"LimeSDR not available: {e}\n  - Plugged into a BLUE (USB 3.0) port?  - Run: SoapySDRUtil --find")
        sys.exit(1)
    print(f"Radio: {rx.label}  antenna input {rx.antenna}  gain {a.gain} dB  (RECEIVE ONLY)  "
          f"chip temperature {rx.temp_c():.1f} C")
    towers = load_towers()
    clip = False

    # ---- stage 1: power sweep on the 100 kHz LTE raster
    if a.freq:
        cands = [a.freq]
        if hasattr(rx, "walk_in") and not a.no_walk:
            print(f"  tuning to {a.freq} MHz in 0.1 MHz steps from 869.7 (like the full scan)...")
            rx.walk_in(a.freq)
    else:
        freqs = np.round(np.arange(a.start, a.stop + 1e-6, 0.1), 1)
        pw = []
        print(f"Stage 1/2: power sweep {freqs[0]:.1f}-{freqs[-1]:.1f} MHz ({len(freqs)} steps)")
        for i, f in enumerate(freqs):
            x, _ = rx.capture(f, int(0.005 * FS_HW))
            clip |= np.max(np.abs(x)) > 0.95
            pw.append(power_dbfs(x))
            if i % 25 == 0:
                print(f"  {f:6.1f} MHz  {pw[-1]:6.1f} dBFS")
        pw = np.array(pw)
        floor = np.percentile(pw, 20)
        occupied = pw > floor + 4
        print(f"  noise floor ~{floor:.1f} dBFS, {occupied.sum()} raster points with signal")
        if occupied.sum() == 0:
            print("  No LTE energy found. Check: antenna on the RX port, outdoors / near a window, try --gain 55.")
        cands = list(freqs[occupied])

    # ---- stage 2: PSS check at each candidate, full ID decode where a PSS shows up
    print(f"Stage 2/2: looking for LTE sync signals at {len(cands)} frequencies")
    best = {}
    t0 = time.time()
    for i, f in enumerate(cands):
        tries = 3 if a.freq else 1
        for k in range(tries):
            x, t_ns = rx.capture(f, int(a.ms / 1000 * FS_HW))
            clip |= np.max(np.abs(x)) > 0.95
            if a.freq:
                x192d = L.to_1p92(x.astype(complex), FS_HW)
                print(f"  try {k + 1}: temp {rx.temp_c():.1f} C, power {power_dbfs(x):6.1f} dBFS, peak {np.max(np.abs(x)):.2f} FS, "
                      f"best sync {quick_pss_db(x192d[:int(0.02 * L.FS)]):5.1f} dB")
                if a.debug:
                    os.makedirs(a.out, exist_ok=True)
                    npy = os.path.join(a.out, f"iq_{f:.1f}MHz_g{a.gain:.0f}_try{k + 1}.npy")
                    np.save(npy, x192d.astype(np.complex64))
                    print(f"         saved raw samples (1.92 MS/s) -> {npy}")
                    for c in L.detect_cells(x192d, min_pss_db=4, min_sss_rho=0.0, max_cells=3, coarse=True)[:6]:
                        print(f"         candidate PCI {c['pci']:3d}  pss {float(c['pss_db']):5.1f} dB  "
                              f"sss q {c['sss_rho']:5.2f}  cfo {c['cfo_hz']:7.0f} Hz  pos {c['toa_samples']}")
                found = L.detect_cells(x192d, coarse=True)
                if found:
                    break
        x192 = L.to_1p92(x.astype(complex), FS_HW)
        if not a.freq and quick_pss_db(x192[:int(0.02 * L.FS)]) < 11:
            continue
        if a.debug and not a.freq:
            os.makedirs(a.out, exist_ok=True)
            np.save(os.path.join(a.out, f"iq_scan_{f:.1f}MHz_g{a.gain:.0f}.npy"), x192.astype(np.complex64))
        for c in L.detect_cells(x192, coarse=bool(a.freq)):
            c.update(freq_mhz=float(f), earfcn=earfcn(f), power_dbfs=round(power_dbfs(x192), 1))
            old = best.get(c["pci"])
            if old is None or c["pss_db"] > old["pss_db"]:
                best[c["pci"]] = c
        if (i + 1) % 20 == 0:
            print(f"  {i + 1}/{len(cands)} checked, {len(best)} cells so far ({time.time() - t0:.0f} s)")
    print(f"  chip temperature at the end: {rx.temp_c():.1f} C")
    rx.close()

    cells = sorted(best.values(), key=lambda c: -c["pss_db"])
    print("\n  MHz     EARFCN  PCI  N_ID1  N_ID2  PSS dB  SSS q   CFO Hz   tower")
    for c in cells:
        print(f"  {c['freq_mhz']:6.1f}  {c['earfcn']:5d}  {c['pci']:4d}  {c['nid1']:4d}   {c['nid2']:3d}   "
              f"{float(c['pss_db']):5.1f}   {c['sss_rho']:.2f}  {c['cfo_hz']:7.0f}   {towers.get(c['nid1'], '?')}")
    if not cells:
        print("  (no cells decoded)")
    sites = {}
    for c in cells:
        sites.setdefault(c["nid1"], []).append(c["pci"])
    if sites:
        print("\n  Sites (same N_ID1 = same tower, different sectors):")
        for n1, p in sorted(sites.items(), key=lambda kv: -max(best[x]["pss_db"] for x in kv[1])):
            print(f"    N_ID1 {n1:3d}: PCIs {sorted(p)}   -> {towers.get(n1, 'unknown: fill nid1 in lte_towers.csv')}")
    if clip and not a.sim:
        print("\n  CLIPPING: signal too strong, run again with a lower gain, e.g.  --gain 25")

    os.makedirs(a.out, exist_ok=True)
    name = f"lte_scan_{datetime.now().strftime('%Y%m%d_%H%M%S')}" + (f"_{a.label}" if a.label else "") + ".csv"
    path = os.path.join(a.out, name)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["freq_mhz", "earfcn", "pci", "nid1", "nid2", "pss_db", "sss_quality", "cfo_hz",
                    "power_dbfs", "gain_db", "label", "sim"])
        for c in cells:
            w.writerow([c["freq_mhz"], c["earfcn"], c["pci"], c["nid1"], c["nid2"], c["pss_db"], c["sss_rho"],
                        c["cfo_hz"], c["power_dbfs"], a.gain, a.label, int(a.sim)])
    print(f"\nSaved {path}")


if __name__ == "__main__":
    main()
