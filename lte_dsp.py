"""LTE downlink sync-signal detection (PSS + SSS), receive-side only. numpy (+ scipy for resampling).

Works on complex baseband sampled at 1.92 MS/s (the 128-point FFT grid, 15 kHz subcarriers),
centred on an LTE carrier. Sync signals sit in the central 62 subcarriers whatever the channel width.
FDD, normal cyclic prefix (all Canadian Band 5 LTE).

    from lte_dsp import detect_cells
    cells = detect_cells(iq_1p92)   # list of dicts: pci, nid1, nid2, toa_samples, pss_db, sss_rho, cfo_hz, first_subframe
"""
import numpy as np

FS = 1.92e6
NFFT = 128
HALF_FRAME = 9600                 # 5 ms at 1.92 MS/s: PSS/SSS repeat period
CP_NORMAL = 9                     # CP of symbols 1..6 at 1.92 MS/s
PSS_ROOTS = (25, 29, 34)          # N_ID2 = 0, 1, 2


def _map62(d):
    """62 sync symbols -> 128-bin FFT grid (DC unused): d[0:31] -> k=-31..-1, d[31:62] -> k=1..31."""
    X = np.zeros(NFFT, complex)
    k = np.r_[np.arange(-31, 0), np.arange(1, 32)]
    X[k % NFFT] = d
    return X


def _bins62():
    return np.r_[np.arange(-31, 0), np.arange(1, 32)] % NFFT


def pss_freq(nid2):
    u = PSS_ROOTS[nid2]
    n = np.arange(62)
    d = np.where(n < 31, np.exp(-1j * np.pi * u * n * (n + 1) / 63),
                 np.exp(-1j * np.pi * u * (n + 1) * (n + 2) / 63))
    return d


def pss_time(nid2):
    return np.fft.ifft(_map62(pss_freq(nid2))) * np.sqrt(NFFT)


def _mseq(taps):
    x = [0, 0, 0, 0, 1]
    for i in range(26):
        x.append(sum(x[i + t] for t in taps) % 2)
    return 1 - 2 * np.array(x)


_S = _mseq((2, 0))        # x(i+5) = x(i+2) + x(i)
_C = _mseq((3, 0))        # x(i+5) = x(i+3) + x(i)
_Z = _mseq((4, 2, 1, 0))  # x(i+5) = x(i+4) + x(i+2) + x(i+1) + x(i)


def sss_freq(nid1, nid2, subframe):
    """3GPP TS 36.211 6.11.2 secondary sync sequence (62 values, +-1)."""
    qp = nid1 // 30
    q = (nid1 + qp * (qp + 1) // 2) // 30
    mp = nid1 + q * (q + 1) // 2
    m0 = mp % 31
    m1 = (m0 + mp // 31 + 1) % 31
    n = np.arange(31)
    s0, s1 = _S[(n + m0) % 31], _S[(n + m1) % 31]
    c0, c1 = _C[(n + nid2) % 31], _C[(n + nid2 + 3) % 31]
    z0, z1 = _Z[(n + m0 % 8) % 31], _Z[(n + m1 % 8) % 31]
    d = np.empty(62)
    if subframe == 0:
        d[0::2], d[1::2] = s0 * c0, s1 * c1 * z0
    else:
        d[0::2], d[1::2] = s1 * c0, s0 * c1 * z1
    return d


_SSS_TABLE = {}


def _sss_table(nid2):
    if nid2 not in _SSS_TABLE:
        _SSS_TABLE[nid2] = np.array([[sss_freq(n1, nid2, sf) for n1 in range(168)] for sf in (0, 5)])
    return _SSS_TABLE[nid2]          # shape (2, 168, 62)


def to_1p92(iq, fs_in):
    """Resample to 1.92 MS/s (integer ratio fs_in / 1.92e6 expected, e.g. 3.84 or 7.68 MS/s)."""
    if abs(fs_in - FS) < 1:
        return iq
    from scipy.signal import resample_poly
    r = int(round(fs_in / FS))
    return resample_poly(iq, 1, r)


CFO_GRID_HZ = np.arange(-25000, 25001, 2500)
COARSE_HZ = (0.0, -100e3, 100e3, -200e3, 200e3)     # whole-raster tuning errors seen on the real LimeSDR   # LimeSDR LO error seen up to ~11 ppm (10 kHz) on real data


def pss_search(iq, nid2):
    """Coherent 128-sample PSS correlation, NORMALISED by the energy in the same window
    (so bursty LTE traffic does not create fake peaks), over a grid of frequency offsets.
    Returns (metric folded over 5 ms [9600], best CFO per position [9600]). Metric ~1 for a clean PSS,
    ~1/128 for random data."""
    from scipy.signal import fftconvolve
    ref = pss_time(nid2)
    e = np.convolve(np.abs(iq) ** 2, np.ones(NFFT), mode="valid") * NFFT + 1e-20
    n = len(iq) - NFFT + 1
    periods = max(1, n // HALF_FRAME)
    t = np.arange(NFFT) / FS
    best_m, best_f = None, None
    for f in CFO_GRID_HZ:
        h = np.conj(ref * np.exp(2j * np.pi * f * t))[::-1]
        c = np.abs(fftconvolve(iq, h, mode="valid")) ** 2 / e
        m = c[:periods * HALF_FRAME].reshape(periods, HALF_FRAME).mean(0)
        if best_m is None:
            best_m, best_f = m, np.full(HALF_FRAME, f, float)
        else:
            better = m > best_m
            best_m = np.where(better, m, best_m)
            best_f = np.where(better, f, best_f)
    return best_m, best_f


def _cfo_from_pss(iq, starts, nid2):
    """Fine CFO: phase change between PSS halves, summed coherently over all repetitions."""
    ref = pss_time(nid2)
    acc = 0j
    for st in starts:
        x = iq[st:st + NFFT] * np.conj(ref)
        acc += x[NFFT // 2:].sum() * np.conj(x[:NFFT // 2].sum())
    return np.angle(acc) / (2 * np.pi * (NFFT // 2) / FS)


def detect_cells(iq, min_pss_db=10.0, max_cells=6, min_sss_rho=0.4, min_pss_rho2=0.04, coarse=False):
    """Find LTE cells in >= 10 ms of 1.92 MS/s baseband. Strongest first.
    coarse=True also tries +-100/200 kHz tuning errors (single-frequency mode)."""
    if coarse:
        n = np.arange(len(iq))
        out = {}
        for c in COARSE_HZ:
            y = np.asarray(iq, complex) * np.exp(-2j * np.pi * c * n / FS)
            for cell in detect_cells(y, min_pss_db, max_cells, min_sss_rho, min_pss_rho2, coarse=False):
                cell["cfo_hz"] = round(float(cell["cfo_hz"] + c), 1)
                if cell["pci"] not in out or cell["sss_rho"] > out[cell["pci"]]["sss_rho"]:
                    out[cell["pci"]] = cell
        return sorted(out.values(), key=lambda c: -float(c["pss_db"]))
    iq = np.asarray(iq, complex)
    iq = iq - iq.mean()
    found = []
    for nid2 in range(3):
        m, fgrid = pss_search(iq, nid2)
        floor = np.median(m) + 1e-30
        work = m.copy()
        for _ in range(max_cells):
            k = int(np.argmax(work))
            pss_db = 10 * np.log10(work[k] / floor)
            if pss_db < min_pss_db or work[k] < min_pss_rho2:
                break
            work[max(0, k - 3):k + 4] = 0
            found.append((pss_db, nid2, k, fgrid[k]))
    cells = []
    for pss_db, nid2, k, f0 in sorted(found, reverse=True):
        # average over all 5 ms repetitions in the capture
        starts = [k + p * HALF_FRAME for p in range((len(iq) - NFFT) // HALF_FRAME)
                  if k + p * HALF_FRAME - (NFFT + CP_NORMAL) >= 0 and k + p * HALF_FRAME + NFFT <= len(iq)]
        if not starts:
            continue
        t_all = np.arange(len(iq))
        derot = iq * np.exp(-2j * np.pi * f0 * t_all / FS)              # coarse CFO from the grid
        fine = _cfo_from_pss(derot, starts, nid2)
        cfo = f0 + (fine if abs(fine) < 2000 else 0.0)                 # fine CFO (keep grid if implausible)
        tbl = _sss_table(nid2)
        scores = np.zeros((2, 168))
        zs = []
        bins = _bins62()
        P = pss_freq(nid2)
        for j, s in enumerate(starts):
            t = np.arange(s - NFFT - CP_NORMAL, s + NFFT)
            seg = iq[t] * np.exp(-2j * np.pi * cfo * t / FS)
            Ys = np.fft.fft(seg[:NFFT])[bins]                  # SSS symbol (one symbol before PSS)
            Yp = np.fft.fft(seg[NFFT + CP_NORMAL:])[bins]      # PSS symbol
            H = Yp * np.conj(P)                                # channel estimate from PSS
            z = np.real(Ys * np.conj(H))                       # coherent SSS observation
            zs.append(z)
            sc = tbl @ z                                       # (2,168) correlation
            # half-frames alternate subframe 0 / 5: align hypotheses to the first one
            scores += sc if j % 2 == 0 else sc[::-1]
        sf_idx, nid1 = divmod(int(np.argmax(scores)), 168)   # equalised -> correct one is positive
        # quality = mean correlation coefficient between each observed SSS and the chosen sequence
        # (1 = perfect, wrong-ID cross-correlation is ~0.5 at most, noise ~0)
        # (averaging the even and the odd half-frames first suppresses interference from other cells)
        rhos = []
        for par in (0, 1):
            zz = np.sum(zs[par::2], axis=0) if zs[par::2] else None
            if zz is None:
                continue
            ref = tbl[sf_idx if par == 0 else 1 - sf_idx, nid1]
            rhos.append(zz @ ref / (np.linalg.norm(zz) * np.sqrt(62) + 1e-30))
        rho = float(np.mean(rhos))
        if rho < min_sss_rho:
            continue
        pci = 3 * nid1 + nid2
        if any(c["pci"] == pci for c in cells):
            continue
        cells.append({"pci": pci, "nid1": nid1, "nid2": nid2, "toa_samples": k,
                      "pss_db": round(pss_db, 1), "sss_rho": round(rho, 2),
                      "cfo_hz": round(float(cfo), 1), "first_subframe": 0 if sf_idx == 0 else 5})
    return cells


# ------------------------------------------------------------------ synthetic test signal
def synth_cell(n_samples, pci, delay, cfo_hz=0.0, amp=1.0, rng=None):
    """Very simplified LTE downlink: PSS + SSS every 5 ms plus random 'data' OFDM elsewhere."""
    rng = rng or np.random.default_rng()
    nid1, nid2 = divmod(pci, 3)
    sym_len = NFFT + CP_NORMAL
    out = np.zeros(n_samples + 2 * HALF_FRAME, complex)
    pss = _map62(pss_freq(nid2))
    for hf in range(len(out) // HALF_FRAME):
        base = hf * HALF_FRAME
        for s in range(70):                                   # 70 symbols per 5 ms (approx. CP mix)
            st = base + s * sym_len - 1 if s else base
            if st + sym_len > len(out):
                break
            if s == 68:
                X = pss
            elif s == 67:
                X = _map62(sss_freq(nid1, nid2, 0 if hf % 2 == 0 else 5))
            else:
                X = np.zeros(NFFT, complex)
                X[np.r_[1:37, 92:128]] = (rng.choice([-1, 1], 72) + 1j * rng.choice([-1, 1], 72)) / np.sqrt(2)
            x = np.fft.ifft(X) * np.sqrt(NFFT)
            out[st:st + sym_len] = np.r_[x[-CP_NORMAL:], x]
    out = np.roll(out, delay)[HALF_FRAME:HALF_FRAME + n_samples]
    t = np.arange(n_samples)
    return amp * out * np.exp(2j * np.pi * cfo_hz * t / FS)


def pss_start_in_synth(delay):
    """Where the PSS useful part starts in synth_cell output (for tests)."""
    st = 68 * (NFFT + CP_NORMAL) - 1 + CP_NORMAL
    return (st + delay - HALF_FRAME) % HALF_FRAME


# ------------------------------------------------------------------ timing (for pseudo-ranges)
def peak_frac(m, k):
    """Parabolic interpolation of a (folded, circular) metric around integer peak k -> fractional index."""
    n = len(m)
    a, b, c = m[(k - 1) % n], m[k], m[(k + 1) % n]
    den = a - 2 * b + c
    d = 0.5 * (a - c) / den if den < 0 else 0.0
    return k + max(-0.5, min(0.5, d))


def _pss_metric_fixed_cfo(iq, nid2, cfo):
    from scipy.signal import fftconvolve
    t = np.arange(NFFT) / FS
    h = np.conj(pss_time(nid2) * np.exp(2j * np.pi * cfo * t))[::-1]
    e = np.convolve(np.abs(iq) ** 2, np.ones(NFFT), mode="valid") * NFFT + 1e-20
    c = np.abs(fftconvolve(iq, h, mode="valid")) ** 2 / e
    periods = max(1, len(c) // HALF_FRAME)
    return c[:periods * HALF_FRAME].reshape(periods, HALF_FRAME).mean(0)


def find_pss_peaks(iq, min_rho2=0.04, min_db=10.0, sep=40, cfo=None):
    """All PSS arrivals in a burst: list of (nid2, k_frac, metric, cfo_hz), strongest first.
    k_frac = position of the PSS useful part within the 5 ms fold (samples at 1.92 MS/s).
    All towers are far more stable than the receiver, so every real cell shows the SAME frequency
    offset: pass 1 finds it on the strongest arrival, pass 2 searches all cells at that one offset
    (this removes the PSS 'ghost' peaks that appear at wrong offset/timing combinations)."""
    iq = np.asarray(iq, complex)
    iq = iq - iq.mean()
    n = np.arange(len(iq))
    if cfo is None:
        best = None
        for coarse in COARSE_HZ:                 # the LimeSDR sometimes lands ~100 kHz off (PLL lock)
            y = iq * np.exp(-2j * np.pi * coarse * n / FS) if coarse else iq
            for nid2 in range(3):
                m, fg = pss_search(y, nid2)
                k = int(np.argmax(m))
                if best is None or m[k] > best[0]:
                    best = (m[k], nid2, k, coarse + fg[k], np.median(m))
        m0, nid2_0, k0, f0, med0 = best
        if m0 < min_rho2 or 10 * np.log10(m0 / (med0 + 1e-30)) < min_db:
            return []
        starts = [k0 + p * HALF_FRAME for p in range(len(iq) // HALF_FRAME) if k0 + p * HALF_FRAME + NFFT <= len(iq)]
        fine = _cfo_from_pss(iq * np.exp(-2j * np.pi * f0 * n / FS), starts, nid2_0)
        cfo = f0 + (fine if abs(fine) < 2000 else 0.0)
    out = []
    for nid2 in range(3):
        m = _pss_metric_fixed_cfo(iq, nid2, cfo)
        med = np.median(m) + 1e-30
        w = m.copy()
        while True:
            k = int(np.argmax(w))
            if w[k] < min_rho2 or 10 * np.log10(w[k] / med) < min_db:
                break
            out.append((nid2, peak_frac(m, k), float(m[k]), float(cfo)))
            w[(np.arange(k - sep, k + sep + 1)) % HALF_FRAME] = 0
    out.sort(key=lambda v: -v[2])
    return out


def sss_decode_at(iq, nid2, k, cfo):
    """Decode N_ID1 for a PSS at integer position k (fold), both methods; returns (nid1, quality) or (None, q)."""
    iq = np.asarray(iq, complex)
    n = np.arange(len(iq))
    xd = iq * np.exp(-2j * np.pi * cfo * n / FS)
    starts = [k + p * HALF_FRAME for p in range(len(iq) // HALF_FRAME + 1)
              if k + p * HALF_FRAME - (NFFT + CP_NORMAL) >= 0 and k + p * HALF_FRAME + NFFT <= len(iq)]
    if len(starts) < 2:
        return None, 0.0
    tbl = _sss_table(nid2)
    bins = _bins62()
    P = pss_freq(nid2)
    zs = []
    for s in starts:
        seg = xd[s - NFFT - CP_NORMAL: s + NFFT]
        Ys = np.fft.fft(seg[:NFFT])[bins]
        Yp = np.fft.fft(seg[NFFT + CP_NORMAL:])[bins]
        zs.append(np.real(Ys * np.conj(Yp * np.conj(P))))
    scores = np.zeros((2, 168))
    for j, z in enumerate(zs):
        sc = tbl @ z
        scores += sc if j % 2 == 0 else sc[::-1]
    sf_idx, nid1 = divmod(int(np.argmax(scores)), 168)
    rhos = []
    for par in (0, 1):
        if zs[par::2]:
            zz = np.sum(zs[par::2], axis=0)
            ref = tbl[sf_idx if par == 0 else 1 - sf_idx, nid1]
            rhos.append(zz @ ref / (np.linalg.norm(zz) * np.sqrt(62) + 1e-30))
    q = float(np.mean(rhos))
    return (nid1, q) if q >= 0.4 else (None, q)
