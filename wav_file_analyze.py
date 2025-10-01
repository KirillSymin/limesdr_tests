#!/usr/bin/env python3
# inspect_wav_iq_min.py
# - Prints header & level stats
# - Works with stereo IQ (L=I, R=Q) OR mono
# - If --assume-mono-iq is set, splits mono as I=even, Q=odd samples
# - Can export a proper stereo 16-bit IQ WAV via --write-stereo

import argparse, sys, numpy as np

def main():
    ap = argparse.ArgumentParser(description="Inspect WAV; handle stereo IQ or mono")
    ap.add_argument("wav", help="Path to WAV file")
    ap.add_argument("--seconds", type=float, default=1.0, help="Seconds to analyze (default 1.0)")
    ap.add_argument("--plot", action="store_true", help="Plot spectrum/constellation (needs matplotlib)")
    ap.add_argument("--assume-mono-iq", action="store_true",
                    help="If WAV is mono: treat even samples as I and odd as Q")
    ap.add_argument("--write-stereo", metavar="OUT.wav",
                    help="If I/Q available, write stereo 16-bit WAV (I=Left, Q=Right)")
    args = ap.parse_args()

    try:
        import soundfile as sf
    except Exception as e:
        print("soundfile not installed. Try: pip install soundfile  (and libsndfile1 via apt)", file=sys.stderr)
        raise

    info = sf.info(args.wav)
    sr = info.samplerate
    ch = info.channels
    frames_total = info.frames
    frames = min(frames_total, int(sr*args.seconds)) if args.seconds else frames_total

    data, _ = sf.read(args.wav, frames=frames, dtype="float32", always_2d=True)  # shape (N, ch)

    print(f"File      : {args.wav}")
    print(f"SampleRate: {sr} Hz")
    print(f"Channels  : {ch}")
    print(f"Frames    : {len(data)} (analyzed ~{len(data)/sr:.3f} s)")
    print(f"Subtype   : {info.subtype or 'n/a'}")
    print()

    def stats(name, x):
        mean = float(np.mean(x))
        pk = float(np.max(np.abs(x)))
        rms = float(np.sqrt(np.mean(x*x) + 1e-30))
        pk_db = 20*np.log10(pk + 1e-30)
        rms_db = 20*np.log10(rms + 1e-30)
        clip = (np.any(x >= 0.999) or np.any(x <= -0.999))
        print(f"{name:5s}: mean={mean:+.5f}, RMS={rms_db:6.2f} dBFS, peak={pk_db:6.2f} dBFS{'  (CLIP)' if clip else ''}")

    if ch == 2:
        L = data[:,0]; R = data[:,1]
        stats("Left", L); stats("Right", R)
        corr = float(np.corrcoef(L, R)[0,1])
        print(f"\nL/R correlation: {corr:+.3f}")

        # Quick spectrum & dominant tone sign using complex x = L + jR
        I, Q = L.astype(np.float64), R.astype(np.float64)
        x = I + 1j*Q
        N = min(len(x), 1<<18)
        if N >= 4096:
            w = 0.5 - 0.5*np.cos(2*np.pi*np.arange(N)/N)
            X = np.fft.fftshift(np.fft.fft(x[:N]*w))
            f = np.fft.fftshift(np.fft.fftfreq(N, d=1/sr))
            mag = np.abs(X)
            # split pos/neg power (ignore tiny DC band)
            guard = max(2, int(10*sr/N))
            pos = f > (guard*sr/N)
            neg = f < -(guard*sr/N)
            print(f"Pos/Neg power: {float(np.sum(mag[pos]**2)):.3e} / {float(np.sum(mag[neg]**2)):.3e}")
            print("Hint: if spectrum looks mirrored (tone below LO instead of above), swap I/Q or flip Q.")
        else:
            print("Note: chunk too short for spectrum summary.")
        if args.write_stereo:
            sf.write(args.write_stereo, data, sr, subtype="PCM_16")
            print(f"\nWrote stereo IQ WAV -> {args.write_stereo}")

    elif ch == 1:
        m = data[:,0]
        stats("Mono", m)

        if args.assume-mono-iq:
            # Split even/odd into I/Q
            N = len(m) - (len(m) % 2)
            I = m[:N:2]
            Q = m[1:N:2]
            print("\nAssuming mono is interleaved IQ: I=even, Q=odd samples")
            stats("I(even)", I); stats("Q(odd)", Q)
            # quick check: RMS similarity and low corr suggest plausible IQ
            rmsI = float(np.sqrt(np.mean(I*I)+1e-30))
            rmsQ = float(np.sqrt(np.mean(Q*Q)+1e-30))
            corr = float(np.corrcoef(I, Q)[0,1]) if len(I) > 10 else 0.0
            print(f"RMS(dB) I/Q: {20*np.log10(rmsI+1e-30):.2f} / {20*np.log10(rmsQ+1e-30):.2f}   corr={corr:+.3f}")

            # spectrum summary
            x = I.astype(np.float64) + 1j*Q.astype(np.float64)
            Nfft = min(len(x), 1<<18)
            if Nfft >= 4096:
                w = 0.5 - 0.5*np.cos(2*np.pi*np.arange(Nfft)/Nfft)
                X = np.fft.fftshift(np.fft.fft(x[:Nfft]*w))
                f = np.fft.fftshift(np.fft.fftfreq(Nfft, d=1/sr))
                mag = np.abs(X)
                guard = max(2, int(10*sr/Nfft))
                pos = f > (guard*sr/Nfft)
                neg = f < -(guard*sr/Nfft)
                print(f"Pos/Neg power: {float(np.sum(mag[pos]**2)):.3e} / {float(np.sum(mag[neg]**2)):.3e}")
            else:
                print("Note: chunk too short for spectrum summary.")

            if args.write_stereo:
                # Write proper stereo 16-bit IQ WAV for LimeSDR TX
                stereo = np.stack([I, Q], axis=1)
                # Convert to int16 safely
                s = np.clip(stereo, -1.0, 1.0)
                s16 = (s * 32767.0).astype(np.int16)
                sf.write(args.write_stereo, s16, sr, subtype="PCM_16")
                print(f"\nWrote stereo IQ WAV -> {args.write_stereo}")
        else:
            print("\nMono file detected.")
            print("If this is real (non-IQ) audio, you cannot recover Q.")
            print("If it is actually IQ packed as I0,Q0,I1,Q1,... in mono, re-run with --assume-mono-iq")
    else:
        print(f"Unsupported channel count: {ch}")

    if args.plot:
        try:
            import matplotlib.pyplot as plt
            if ch == 2:
                I, Q = data[:,0], data[:,1]
            elif ch == 1 and args.assume-mono-iq:
                N = len(data); N -= (N % 2)
                I, Q = data[:N:2,0], data[1:N:2,0]
            else:
                I, Q = None, None

            if I is not None:
                x = I + 1j*Q
                M = min(len(x), 1<<18)
                w = 0.5 - 0.5*np.cos(2*np.pi*np.arange(M)/M)
                X = np.fft.fftshift(np.fft.fft(x[:M]*w))
                f = np.fft.fftshift(np.fft.fftfreq(M, d=1/sr))
                plt.figure(); plt.plot(f/1e3, 20*np.log10(np.abs(X)+1e-12))
                plt.xlabel("Frequency (kHz)"); plt.ylabel("Magnitude (dB)"); plt.title("Spectrum")
                step = max(1, len(x)//5000)
                xn = x[::step] / (np.max(np.abs(x[::step])) + 1e-12)
                plt.figure(); plt.plot(np.real(xn), np.imag(xn), '.', markersize=2)
                plt.xlabel("I"); plt.ylabel("Q"); plt.title("Constellation"); plt.axis('equal')
                plt.show()
            else:
                # Real-only spectrum
                M = min(len(data[:,0]), 1<<18)
                w = 0.5 - 0.5*np.cos(2*np.pi*np.arange(M)/M)
                X = np.fft.fftshift(np.fft.fft(data[:M,0]*w))
                f = np.fft.fftshift(np.fft.fftfreq(M, d=1/sr))
                import matplotlib.pyplot as plt
                plt.figure(); plt.plot(f/1e3, 20*np.log10(np.abs(X)+1e-12))
                plt.xlabel("Frequency (kHz)"); plt.ylabel("Magnitude (dB)"); plt.title("Real signal spectrum")
                plt.show()
        except Exception as e:
            print(f"(plot skipped: {e})", file=sys.stderr)

if __name__ == "__main__":
    main()
