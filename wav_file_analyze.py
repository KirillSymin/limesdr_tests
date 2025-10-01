#!/usr/bin/env python3
import argparse, sys, math, numpy as np

def try_read_wav(path, seconds=None):
    """
    Read WAV using soundfile (if installed), else scipy.io.wavfile.
    Returns (sr, data_float32[N,2]) and dtype string.
    """
    try:
        import soundfile as sf
        # With SoundFile, frames ~ samples-per-channel; data comes as shape (frames, channels)
        info = sf.info(path)
        sr = info.samplerate
        frames = info.frames
        ch = info.channels
        subtype = info.subtype or "unknown"
        if seconds is not None:
            frames = min(frames, int(seconds*sr))
        data, _ = sf.read(path, frames=frames, dtype='float32', always_2d=True)
        if data.shape[1] != 2:
            raise ValueError(f"WAV must be stereo; got {data.shape[1]} channels")
        return sr, data.astype(np.float32, copy=False), f"libsndfile/{subtype}"
    except Exception as e_sf:
        try:
            from scipy.io import wavfile
            sr, data = wavfile.read(path)  # data: (N,2) for stereo; dtype depends on file
            if data.ndim == 1:
                raise ValueError("WAV must be stereo; got mono")
            if data.shape[1] != 2:
                raise ValueError(f"WAV must be stereo; got {data.shape[1]} channels")
            if seconds is not None:
                data = data[:int(seconds*sr), :]
            # Normalize to float32 in [-1,1] for analysis
            if np.issubdtype(data.dtype, np.floating):
                out = np.clip(data.astype(np.float32), -1.0, 1.0)
                kind = f"scipy/{data.dtype}"
            elif data.dtype == np.int16:
                out = (data.astype(np.float32) / 32768.0)
                kind = "scipy/int16"
            elif data.dtype == np.int32:
                out = (data.astype(np.float32) / 2147483648.0)
                kind = "scipy/int32"
            else:
                # Fallback: scale by max
                scale = float(np.iinfo(data.dtype).max)
                out = (data.astype(np.float32) / scale)
                kind = f"scipy/{data.dtype}"
            return sr, out, kind
        except Exception as e_sp:
            raise RuntimeError(f"Failed to read WAV with soundfile ({e_sf}) and scipy ({e_sp}).")

def rms_dbfs(x):
    x = np.asarray(x, dtype=np.float64)
    rms = np.sqrt(np.mean(x*x) + 1e-30)
    return 20*np.log10(rms + 1e-30), rms

def peak_dbfs(x):
    x = np.asarray(x, dtype=np.float64)
    pk = np.max(np.abs(x))
    return 20*np.log10(pk + 1e-30), pk

def quick_fft_metrics(I, Q, sr, pilot_hz=None):
    """Return dict with DC power, pos/neg power, dominant bin freq sign, mapping guess scores."""
    x = I.astype(np.float64) + 1j*Q.astype(np.float64)
    N = int(1<<int(np.floor(np.log2(min(len(x), 1<<20)))))  # up to ~1M, power of two
    x = x[:N]
    if N < 4096:
        return {"note": "too short for spectral metrics"}
    # Hann window to tame leakage
    w = 0.5 - 0.5*np.cos(2*np.pi*np.arange(N)/N)
    X = np.fft.fft(x*w)
    freqs = np.fft.fftfreq(N, d=1.0/sr)
    mag = np.abs(X)

    # DC power (first bin)
    dc_pow = (mag[0]**2)

    # Positive/negative frequency power (exclude a tiny DC band)
    guard = max(2, int(10*sr/N))  # ~10 Hz guard
    pos_mask = freqs > (guard*sr/N)
    neg_mask = freqs < -(guard*sr/N)
    pos_pow = float(np.sum(mag[pos_mask]**2))
    neg_pow = float(np.sum(mag[neg_mask]**2))

    # Dominant tone ignoring DC and its immediate neighbors
    mask = np.ones_like(mag, dtype=bool)
    mask[:guard] = False
    mask[-guard:] = False
    k = int(np.argmax(mag[mask]))
    # map back to true index
    real_idx = np.arange(len(mag))[mask][k]
    dom_freq = float(freqs[real_idx])

    # If a pilot is provided, check energy around +/- pilot
    pilot_score = None
    if pilot_hz:
        bw = max(2, int(0.001*N))  # ~0.1% bin span
        def band_power(f0):
            # sum |X| in a small band around f0
            idx = np.argmin(np.abs(freqs - f0))
            lo = max(0, idx-bw); hi = min(len(mag), idx+bw+1)
            return float(np.sum(mag[lo:hi]**2))
        ppos = band_power(+pilot_hz)
        pneg = band_power(-pilot_hz)
        pilot_score = ppos - pneg

    return {
        "N": N, "dc_pow": dc_pow, "pos_pow": pos_pow, "neg_pow": neg_pow,
        "dom_freq": dom_freq, "pilot_score": pilot_score
    }

def guess_iq_mapping(L, R, sr, pilot_hz=None):
    """
    Return ('L_is_I', score) or ('R_is_I', score) or ('inconclusive', 0).
    Heuristic: compare (pos-neg) power when forming complex as L+jR vs R+jL;
    optionally bias with pilot if provided.
    """
    # normalize just in case
    L = L.astype(np.float64); R = R.astype(np.float64)
    def score(I, Q):
        m = quick_fft_metrics(I, Q, sr, pilot_hz=pilot_hz)
        if "note" in m: return -1e9, m  # too short
        s = (m["pos_pow"] - m["neg_pow"])
        if m["pilot_score"] is not None:
            s += 5.0*m["pilot_score"]  # give pilot a strong say
        # Also nudge based on dominant tone sign
        s += 1e-6 * (m["dom_freq"])  # tiny tie-breaker
        return s, m

    sL, mL = score(L, R)  # assume I=L, Q=R
    sR, mR = score(R, L)  # assume I=R, Q=L

    # confidence = |Δ| / (|sL|+|sR|+ε)
    delta = abs(sL - sR)
    denom = abs(sL) + abs(sR) + 1e-12
    conf = float(delta / denom)

    if conf < 0.02:  # very close -> inconclusive (typical for symmetric OFDM)
        return "inconclusive", conf, mL, mR
    return ("L_is_I" if sL > sR else "R_is_I"), conf, mL, mR

def main():
    ap = argparse.ArgumentParser(description="Inspect a stereo WAV with I/Q data")
    ap.add_argument("wav", help="Path to WAV file")
    ap.add_argument("--seconds", type=float, default=1.0, help="Seconds to analyze (default 1.0)")
    ap.add_argument("--plot", action="store_true", help="Show spectrum & constellation (matplotlib)")
    ap.add_argument("--pilot", type=float, default=None, help="Pilot frequency in Hz (if known)")
    args = ap.parse_args()

    sr, data, kind = try_read_wav(args.wav, seconds=args.seconds)
    N, C = data.shape
    L = data[:,0].copy()
    R = data[:,1].copy()

    # Basic header/format info
    dur = N/sr
    print(f"Reader     : {kind}")
    print(f"SampleRate : {sr} Hz")
    print(f"Channels   : {C} (expect 2: L,R)")
    print(f"Frames     : {N} (~{dur:.3f} s analyzed)")
    print()

    # Level & DC stats
    for name, ch in (("Left", L), ("Right", R)):
        mean = float(np.mean(ch))
        dc_mV = mean*1000.0  # relative to full scale=1.0 -> mFS; for int16 this is m/32768
        pk_db, pk = peak_dbfs(ch)
        rms_db, rms = rms_dbfs(ch)
        clipped = (np.any(ch >= 0.999) or np.any(ch <= -0.999))
        print(f"{name:5s}: mean={mean:+.5f} (~{dc_mV:+.2f} mFS), "
              f"RMS={rms_db:6.2f} dBFS, peak={pk_db:6.2f} dBFS{'  (CLIPPED!)' if clipped else ''}")

    # Cross-channel correlation (quick sanity)
    corr = float(np.corrcoef(L, R)[0,1])
    print(f"\nL/R correlation: {corr:+.3f} (ideal Q is ~90° phase; correlation often low)")

    # Mapping guess
    mapping, conf, mL, mR = guess_iq_mapping(L, R, sr, pilot_hz=args.pilot)
    print("\nI/Q mapping guess:")
    if mapping == "inconclusive":
        print("  Inconclusive (spectrum looks symmetric — common for OFDM).")
    else:
        pretty = "I=Left, Q=Right" if mapping=="L_is_I" else "I=Right, Q=Left"
        print(f"  {pretty}  (confidence ~{100*conf:.1f}%)")
    # Print tiny spectrum summary for the chosen mapping (prefer the 'better' side)
    chosen = mL if mapping!="R_is_I" else mR
    if "note" not in chosen:
        print(f"  DC power: {chosen['dc_pow']:.3e}")
        print(f"  Pos/Neg power: {chosen['pos_pow']:.3e} / {chosen['neg_pow']:.3e}")
        print(f"  Dominant bin near: {chosen['dom_freq']:.1f} Hz")

    # Optional plotting
    if args.plot:
        try:
            import matplotlib.pyplot as plt
            # Spectrum (chosen mapping)
            I = L if mapping != "R_is_I" else R
            Q = R if mapping != "R_is_I" else L
            x = I + 1j*Q
            M = min(len(x), 1<<18)
            w = 0.5 - 0.5*np.cos(2*np.pi*np.arange(M)/M)
            X = np.fft.fftshift(np.fft.fft(x[:M]*w))
            f = np.fft.fftshift(np.fft.fftfreq(M, d=1.0/sr))
            plt.figure(); plt.plot(f/1e3, 20*np.log10(np.abs(X)+1e-12))
            plt.xlabel("Frequency (kHz)"); plt.ylabel("Magnitude (dB)"); plt.title("Spectrum")

            # Constellation (downsampled)
            step = max(1, len(x)//5000)
            xi = (x[::step] / (np.max(np.abs(x[::step])) + 1e-12))
            plt.figure(); plt.plot(np.real(xi), np.imag(xi), '.', markersize=2)
            plt.xlabel("I"); plt.ylabel("Q"); plt.title("Constellation (normalized)"); plt.axis('equal')

            plt.show()
        except Exception as e:
            print(f"(plot skipped: {e})", file=sys.stderr)

if __name__ == "__main__":
    main()
