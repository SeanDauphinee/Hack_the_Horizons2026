#!/usr/bin/env python3
"""LTE burst recorder (RECEIVE ONLY). Started by daq_logger.py as a SEPARATE PROCESS so the radio
never slows the IMU loop. Every PERIOD seconds it captures a short burst on each carrier and appends:

    <run>/lte_iq.bin        int16 I,Q interleaved, 1.92 MS/s, one burst after another
    <run>/lte_bursts.csv    burst,time_s,hw_time_ns,freq_mhz,n_samples,byte_offset,gain_db,scale

time_s     = Pi clock seconds since logger start (same time base as imu.csv), first sample of burst
hw_time_ns = LimeSDR sample-counter time of the first sample (used for the ranges)

    python3 lte_capture.py --out RUN_DIR --t0 EPOCH --freqs 885.1,875.2 [--sim]
Stops when RUN_DIR/lte_stop exists, on SIGTERM, or Ctrl+C.
"""
import argparse
import csv
import math
import os
import signal
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lte_dsp as L  # noqa: E402

C = 299792458.0


class SimLTE:
    """Static receiver at the sim GPS origin; towers from lte_towers.csv with made-up PCIs,
    each with its own transmit-time offset; receiver clock fast by 5 ppm (CFO + drift)."""
    label = "SIMULATED"
    PPM = 5.0

    def __init__(self):
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import merge_run as M
        self.rng = np.random.default_rng(11)
        origin = (45.38333333, -75.69850000, 51.0)
        towers = []
        with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "lte_towers.csv")) as f:
            for t in csv.DictReader(f):
                h = float(t["ground_msl_m"]) + float(t["antenna_agl_m"]) + M.GEOID_OTTAWA_M
                towers.append(np.array(M.ned(float(t["lat_deg"]), float(t["lon_deg"]), h, origin)))
        self.towers = towers
        # (carrier MHz) -> list of (pci, tower index, tx_offset_s, amp)   receiver walks east 1.389 m/s
        self.cells = {885.1: [(412, 0, 1.234e-3, 1.0), (148, 2, 3.21e-3, 0.5)],
                      876.1: [(110, 1, 0.77e-3, 0.8)]}
        self.t_start = time.time()

    def capture(self, f_mhz, n_out):
        """Returns (iq at 1.92 MS/s, hw_time_ns of first sample)."""
        true_t = time.time() - self.t_start                               # ONE clock read (sim precision)
        true_t = round(true_t * L.FS) / L.FS                              # start on a sample boundary
        hw_t = true_t * (1 + self.PPM * 1e-6)                              # receiver clock runs fast
        x = (self.rng.standard_normal(n_out) + 1j * self.rng.standard_normal(n_out)) * 0.02
        rx_pos = np.array([1.389 * max(0.0, true_t - 35), 1.389 * min(true_t, 35), 0.0])   # L-walk (N, E, D)
        for pci, ti, txo, amp in self.cells.get(round(f_mhz, 1), []):
            rng_m = float(np.linalg.norm(self.towers[ti] - rx_pos))
            # PSS useful part starts at true times txo + k*5ms (+ propagation); position in this burst:
            arr = (txo + rng_m / C - true_t) % 0.005                      # seconds after burst start
            d = arr * L.FS
            base = L.synth_cell(n_out, pci, 0, 0.0, amp * 0.3, self.rng)
            k0 = L.pss_start_in_synth(0)
            X = np.fft.fft(base)
            fr = np.fft.fftfreq(n_out)
            sh = np.fft.ifft(X * np.exp(-2j * np.pi * fr * (d - k0)))      # fractional delay
            cfo = -self.PPM * 1e-6 * f_mhz * 1e6
            x = x + sh * np.exp(2j * np.pi * cfo * np.arange(n_out) / L.FS)
        time.sleep(n_out / L.FS)
        return x.astype(np.complex64), int(hw_t * 1e9)

    def close(self):
        pass

    def temp_c(self):
        return 40.0


class RealLTE:
    def __init__(self, gain):
        from lte_scan import LimeRX, FS_HW
        self.rx = LimeRX(gain)
        self.label = self.rx.label
        self.fs_hw = FS_HW
        self.last_f = None

    def walk_in(self, f_mhz):
        self.rx.walk_in(f_mhz)

    def capture(self, f_mhz, n_out):
        x, t_ns = self.rx.capture(f_mhz, int(n_out * self.fs_hw / L.FS))
        return L.to_1p92(x.astype(complex), self.fs_hw).astype(np.complex64), t_ns

    def temp_c(self):
        return self.rx.temp_c()

    def close(self):
        self.rx.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--t0", type=float, required=True, help="logger start epoch (time_s = time.time() - t0)")
    ap.add_argument("--freqs", default="885.1,875.2")
    ap.add_argument("--ms", type=float, default=40.0)
    ap.add_argument("--period", type=float, default=1.0)
    ap.add_argument("--gain", type=float, default=30.0)
    ap.add_argument("--sim", action="store_true")
    a = ap.parse_args()
    freqs = [float(v) for v in a.freqs.split(",") if v.strip()]
    stop_file = os.path.join(a.out, "lte_stop")
    stop = {"now": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.update(now=True))
    try:
        dev = SimLTE() if a.sim else RealLTE(a.gain)
    except Exception as e:
        print(f"LTE: radio not available ({e}). Recording continues without LTE.", flush=True)
        with open(os.path.join(a.out, "lte_status.txt"), "w") as f:
            f.write(f"unavailable: {e}\n")
        return
    with open(os.path.join(a.out, "lte_status.txt"), "w") as f:
        f.write(f"ok: {dev.label}, carriers {freqs}, {a.ms} ms every {a.period} s, gain {a.gain}\n")
    if hasattr(dev, "walk_in"):
        dev.walk_in(freqs[0])            # tune like the scan does (direct jumps gave undecodable IDs)
    print(f"LTE: recording {freqs} MHz, {a.ms:.0f} ms bursts every {a.period:.1f} s ({dev.label})", flush=True)
    n = int(a.ms / 1000 * L.FS)
    idx_path = os.path.join(a.out, "lte_bursts.csv")
    new = not os.path.exists(idx_path)
    idx = open(idx_path, "a", buffering=1)
    if new:
        idx.write("burst,time_s,hw_time_ns,freq_mhz,n_samples,byte_offset,gain_db,scale,temp_c\n")
    binf = open(os.path.join(a.out, "lte_iq.bin"), "ab")
    burst = 0
    next_t = time.time()
    try:
        while not stop["now"] and not os.path.exists(stop_file):
            for f_mhz in freqs:
                x, t_ns = dev.capture(f_mhz, n)
                t_end = time.time()
                time_s = t_end - a.t0 - n / L.FS                      # Pi time of the first sample
                scale = float(max(np.max(np.abs(x.real)), np.max(np.abs(x.imag)), 1e-9))
                q = np.empty(2 * n, np.int16)
                q[0::2] = np.round(x.real / scale * 32767)
                q[1::2] = np.round(x.imag / scale * 32767)
                off = binf.tell()
                binf.write(q.tobytes())
                idx.write(f"{burst},{time_s:.4f},{t_ns if t_ns is not None else ''},{f_mhz},{n},{off},{a.gain},{scale:.6g},{dev.temp_c():.1f}\n")
                burst += 1
            next_t += a.period
            time.sleep(max(0.0, next_t - time.time()))
    except KeyboardInterrupt:
        pass
    finally:
        binf.close()
        idx.close()
        dev.close()
        print(f"LTE: {burst} bursts saved", flush=True)


if __name__ == "__main__":
    main()
