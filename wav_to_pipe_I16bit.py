#!/usr/bin/env python3
import argparse
import os
import time
import wave

import numpy as np


def send_iq_to_fifo(fifo_path: str, i: np.ndarray, q: np.ndarray) -> None:
    if i.dtype != np.int16 or q.dtype != np.int16:
        raise ValueError("I and Q must be numpy int16 arrays")
    if i.shape != q.shape:
        raise ValueError("I and Q must have the same shape")

    interleaved = np.empty(i.size * 2, dtype=np.int16)
    interleaved[0::2] = i
    interleaved[1::2] = q

    with open(fifo_path, "wb", buffering=0) as f:
        f.write(interleaved.tobytes())


def stream_wav_to_fifo(wav_path: str, fifo_path: str, chunk_samples: int = 4096, loop: bool = False):
    # Open WAV
    wf = wave.open(wav_path, "rb")
    if wf.getnchannels() != 2:
        raise RuntimeError("WAV must be stereo (2 channels)")
    if wf.getsampwidth() != 2:
        raise RuntimeError("WAV must be 16-bit PCM")

    sr = wf.getframerate()
    print(f"WAV: {sr} Hz, {wf.getnchannels()} ch, 16-bit")

    # C side must be configured with --sample-rate equal to sr
    print(f"Make sure lime_tx_fifo is using --sample-rate {sr}")

    # Open FIFO once and keep it open
    if not os.path.exists(fifo_path):
        raise FileNotFoundError(f"FIFO {fifo_path} does not exist")

    print(f"Opening FIFO {fifo_path} for writing (blocking until C side opens it)...")
    f = open(fifo_path, "wb", buffering=0)
    print("FIFO opened, streaming... (Ctrl+C to stop)")

    total_samples_sent = 0
    t0 = time.perf_counter()

    try:
        while True:
            raw = wf.readframes(chunk_samples)
            if len(raw) == 0:
                if loop:
                    wf.rewind()
                    continue
                else:
                    break

            # Convert bytes → int16
            data = np.frombuffer(raw, dtype=np.int16)

            # Stereo interleaved: L0, R0, L1, R1, ...
            i = data[0::2]
            q = data[1::2]

            # Interleave explicitly (not strictly needed, but matches the helper)
            interleaved = np.empty(i.size * 2, dtype=np.int16)
            interleaved[0::2] = i
            interleaved[1::2] = q

            # Write to FIFO
            f.write(interleaved.tobytes())
            total_samples_sent += i.size

            # Real-time pacing: target time based on sample count
            target_time = total_samples_sent / sr
            now = time.perf_counter()
            sleep_time = t0 + target_time - now
            if sleep_time > 0:
                time.sleep(sleep_time)
    finally:
        f.close()
        wf.close()
        print("Streaming finished.")


def main():
    ap = argparse.ArgumentParser(description="Stream IQ from WAV to FIFO")
    ap.add_argument("--wav", required=True, help="Input stereo 16-bit WAV (I=left, Q=right)")
    ap.add_argument("--fifo", required=True, help="FIFO path (same as --fifo in lime_tx_fifo)")
    ap.add_argument("--chunk", type=int, default=4096, help="Chunk size in samples per channel")
    ap.add_argument("--loop", action="store_true", help="Loop WAV endlessly")
    args = ap.parse_args()

    stream_wav_to_fifo(args.wav, args.fifo, chunk_samples=args.chunk, loop=args.loop)


if __name__ == "__main__":
    main()
