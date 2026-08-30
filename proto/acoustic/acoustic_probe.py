#!/usr/bin/env -S uv run
# /// script
# requires-python = ">=3.11"
# dependencies = [
#   "matplotlib>=3.9",
#   "numpy>=2.0",
#   "pyserial>=3.5",
#   "sounddevice>=0.5",
# ]
# ///
"""THROWAWAY Mac probe for the TABLE-ACOUSTIC audio pipeline.

Usage:
  uv run proto/acoustic/acoustic_probe.py --emit
  uv run proto/acoustic/acoustic_probe.py --emit --f-left 20500 --f-right 21500 --amp 0.05
  uv run proto/acoustic/acoustic_probe.py --sweep
  uv run proto/acoustic/acoustic_probe.py --capture capture.bin --port /dev/cu.usbmodem101
  uv run proto/acoustic/acoustic_probe.py --parse capture.bin [--plot]
  uv run proto/acoustic/acoustic_probe.py --selftest

Capture writes the serial stream byte-for-byte. Parsing understands interleaved
IMU/CFG text lines and AUD headers followed by exact-length binary PCM payloads.
Phase-based tracking is deliberately out of scope; this validates transport,
timing, sustained IMU rate, and near-ultrasonic spectral SNR.
"""

from __future__ import annotations

import argparse
import re
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

PCM_RATE_HZ = 48_000.0
DEFAULT_LEFT_HZ = 21_500.0
DEFAULT_RIGHT_HZ = 22_500.0
DEFAULT_AMP = 0.05
DEFAULT_PORT = "/dev/cu.usbmodem101"
AUD_HEADER = re.compile(rb"AUD,(\d+),(\d+)")


@dataclass
class AudioChunk:
    t_us: int
    samples: np.ndarray


@dataclass
class ParsedCapture:
    pcm: np.ndarray
    chunks: list[AudioChunk]
    imu_t_us: np.ndarray
    malformed_headers: int
    truncated_payloads: int


@dataclass
class TimingReport:
    gaps: list[tuple[int, float, int]]
    inferred_missing_samples: int
    effective_rate_hz: float
    drift_ppm: float
    coverage_percent: float
    imu_count: int
    imu_rate_hz: float


def warn_about_ultrasound() -> None:
    print("WARNING: Near-ultrasonic tones may be audible or distressing to pets.")
    print("Keep the amplitude low, keep animals away, and stop if anyone reacts.")


def output_device() -> tuple[dict, float]:
    import sounddevice as sd

    device = sd.query_devices(kind="output")
    rate_hz = float(device["default_samplerate"])
    if int(device["max_output_channels"]) < 2:
        raise SystemExit(
            f"default output device {device['name']!r} has fewer than two output channels"
        )
    return device, rate_hz


def resolve_emit_frequencies(
    rate_hz: float, requested_left: float | None, requested_right: float | None
) -> tuple[float, float]:
    left = DEFAULT_LEFT_HZ if requested_left is None else requested_left
    right = DEFAULT_RIGHT_HZ if requested_right is None else requested_right
    if rate_hz <= 44_200 and (requested_left is None or requested_right is None):
        if requested_left is None:
            left = 20_500.0
        if requested_right is None:
            right = 21_500.0
        print(
            f"default output rate is {rate_hz:.0f} Hz; capping unspecified default tones to "
            f"{left:.0f}/{right:.0f} Hz to stay below the {rate_hz / 2:.0f} Hz Nyquist limit"
        )
    guard_hz = 250.0
    if min(left, right) <= 0 or max(left, right) >= rate_hz / 2.0 - guard_hz:
        raise SystemExit(
            f"tones must be positive and at least {guard_hz:.0f} Hz below Nyquist "
            f"({rate_hz / 2.0:.0f} Hz for this device)"
        )
    return left, right


def emit(left_hz: float | None, right_hz: float | None, amp: float) -> None:
    import sounddevice as sd

    device, rate_hz = output_device()
    left_hz, right_hz = resolve_emit_frequencies(rate_hz, left_hz, right_hz)
    warn_about_ultrasound()
    print(
        f"output={device['name']!r}, native rate={rate_hz:.0f} Hz, "
        f"channel 1/left={left_hz:.0f} Hz, channel 2/right={right_hz:.0f} Hz, amp={amp:.3f}"
    )
    print("playing continuously; press Ctrl-C to stop")
    frame_index = 0

    def callback(outdata: np.ndarray, frames: int, _time_info: object, status: object) -> None:
        nonlocal frame_index
        if status:
            print(f"audio callback status: {status}", file=sys.stderr)
        indices = np.arange(frame_index, frame_index + frames, dtype=np.float64)
        outdata[:, 0] = amp * np.sin(2.0 * np.pi * left_hz * indices / rate_hz)
        outdata[:, 1] = amp * np.sin(2.0 * np.pi * right_hz * indices / rate_hz)
        frame_index += frames

    try:
        with sd.OutputStream(
            samplerate=rate_hz, channels=2, dtype="float32", callback=callback
        ):
            while True:
                time.sleep(1.0)
    except KeyboardInterrupt:
        print("stopped")


def sweep(amp: float) -> None:
    import sounddevice as sd

    device, rate_hz = output_device()
    warn_about_ultrasound()
    print(
        f"output={device['name']!r}, native rate={rate_hz:.0f} Hz; "
        "each frequency plays for 2 seconds on both stereo channels"
    )
    frequencies = np.arange(16_000.0, 23_000.1, 500.0)
    playable = frequencies < rate_hz / 2.0 - 250.0
    try:
        for frequency in frequencies[playable]:
            print(f"NOW: {frequency:.0f} Hz", flush=True)
            frame_count = int(round(2.0 * rate_hz))
            indices = np.arange(frame_count, dtype=np.float64)
            tone = amp * np.sin(2.0 * np.pi * frequency * indices / rate_hz)
            stereo = np.column_stack((tone, tone)).astype(np.float32)
            sd.play(stereo, samplerate=rate_hz, blocking=True)
    except KeyboardInterrupt:
        sd.stop()
        print("stopped")
        return
    skipped = frequencies[~playable]
    if len(skipped):
        print(
            f"skipped {skipped[0]:.0f}-{skipped[-1]:.0f} Hz because this device's "
            f"Nyquist limit is {rate_hz / 2.0:.0f} Hz"
        )


def capture(port: str, output: Path, baud: int, seconds: float) -> None:
    import serial

    print(f"capturing raw mixed serial bytes from {port} at {baud} baud to {output}")
    print("press Ctrl-C to stop")
    started = time.monotonic()
    byte_count = 0
    try:
        with serial.Serial(port, baudrate=baud, timeout=0.2) as device, output.open(
            "wb"
        ) as sink:
            while seconds <= 0 or time.monotonic() - started < seconds:
                block = device.read(65_536)
                if not block:
                    continue
                sink.write(block)
                byte_count += len(block)
                sink.flush()
    except KeyboardInterrupt:
        pass
    print(f"captured {byte_count} raw bytes to {output}")


def parse_capture(path: Path) -> ParsedCapture:
    raw = path.read_bytes()
    chunks: list[AudioChunk] = []
    imu_t_us: list[int] = []
    malformed_headers = 0
    truncated_payloads = 0
    position = 0

    while position < len(raw):
        newline = raw.find(b"\n", position)
        if newline < 0:
            break
        line = raw[position:newline].rstrip(b"\r")
        position = newline + 1
        if line.startswith(b"AUD,"):
            match = AUD_HEADER.fullmatch(line)
            if not match:
                malformed_headers += 1
                continue
            t_us, n_bytes = (int(value) for value in match.groups())
            payload_end = position + n_bytes
            if payload_end > len(raw):
                truncated_payloads += 1
                break
            payload = raw[position:payload_end]
            position = payload_end
            if n_bytes % 2:
                malformed_headers += 1
                payload = payload[:-1]
            samples = np.frombuffer(payload, dtype="<i2").copy()
            chunks.append(AudioChunk(t_us=t_us, samples=samples))
            continue
        if line.startswith(b"IMU,"):
            fields = line.split(b",")
            if len(fields) == 8:
                try:
                    imu_t_us.append(int(fields[1]))
                except ValueError:
                    pass

    pcm = (
        np.concatenate([chunk.samples for chunk in chunks])
        if chunks
        else np.empty(0, dtype=np.int16)
    )
    return ParsedCapture(
        pcm=pcm,
        chunks=chunks,
        imu_t_us=np.asarray(imu_t_us, dtype=np.int64),
        malformed_headers=malformed_headers,
        truncated_payloads=truncated_payloads,
    )


def analyze_timing(parsed: ParsedCapture) -> TimingReport:
    gaps: list[tuple[int, float, int]] = []
    inferred_missing = 0
    continuous_samples = 0
    continuous_time_s = 0.0

    for index in range(1, len(parsed.chunks)):
        previous = parsed.chunks[index - 1]
        current = parsed.chunks[index]
        delta_s = (current.t_us - previous.t_us) * 1e-6
        expected_s = len(current.samples) / PCM_RATE_HZ
        error_s = delta_s - expected_s
        if delta_s <= 0 or abs(error_s) > max(0.001, expected_s * 0.05):
            missing = max(0, round(delta_s * PCM_RATE_HZ) - len(current.samples))
            gaps.append((index, error_s * 1000.0, missing))
            inferred_missing += missing
        else:
            continuous_samples += len(current.samples)
            continuous_time_s += delta_s

    if continuous_time_s:
        effective_rate = continuous_samples / continuous_time_s
        drift_ppm = (effective_rate / PCM_RATE_HZ - 1.0) * 1e6
    else:
        effective_rate = float("nan")
        drift_ppm = float("nan")

    if parsed.chunks:
        first = parsed.chunks[0]
        audio_start_us = first.t_us - len(first.samples) / PCM_RATE_HZ * 1e6
        audio_end_us = parsed.chunks[-1].t_us
        span_s = (audio_end_us - audio_start_us) * 1e-6
        coverage = 100.0 * len(parsed.pcm) / (span_s * PCM_RATE_HZ) if span_s > 0 else 0.0
        in_audio = parsed.imu_t_us[
            (parsed.imu_t_us >= audio_start_us) & (parsed.imu_t_us <= audio_end_us)
        ]
    else:
        coverage = 0.0
        in_audio = np.empty(0, dtype=np.int64)
    imu_rate = (
        (len(in_audio) - 1) / ((in_audio[-1] - in_audio[0]) * 1e-6)
        if len(in_audio) >= 2 and in_audio[-1] > in_audio[0]
        else float("nan")
    )
    return TimingReport(
        gaps=gaps,
        inferred_missing_samples=inferred_missing,
        effective_rate_hz=effective_rate,
        drift_ppm=drift_ppm,
        coverage_percent=coverage,
        imu_count=len(in_audio),
        imu_rate_hz=imu_rate,
    )


def tone_snrs(
    pcm: np.ndarray, tones_hz: tuple[float, float]
) -> list[tuple[float, float, float]]:
    if len(pcm) < 16:
        return [(tone, float("nan"), float("nan")) for tone in tones_hz]
    samples = pcm.astype(np.float64) / 32768.0
    window = np.hanning(len(samples))
    spectrum = np.fft.rfft(samples * window)
    power = np.abs(spectrum) ** 2
    frequencies = np.fft.rfftfreq(len(samples), 1.0 / PCM_RATE_HZ)
    results: list[tuple[float, float, float]] = []
    for tone in tones_hz:
        search = np.abs(frequencies - tone) <= 5.0
        if not np.any(search):
            results.append((tone, float("nan"), float("nan")))
            continue
        candidate_indices = np.flatnonzero(search)
        tone_index = int(candidate_indices[np.argmax(power[candidate_indices])])
        nearby = np.abs(frequencies - tone) <= 500.0
        for other in tones_hz:
            nearby &= np.abs(frequencies - other) > 25.0
        nearby &= frequencies > 0
        noise_floor = float(np.mean(power[nearby])) if np.any(nearby) else float("nan")
        snr_db = 10.0 * np.log10(power[tone_index] / noise_floor)
        results.append((tone, float(frequencies[tone_index]), float(snr_db)))
    return results


def save_plots(
    path: Path, pcm: np.ndarray, tones_hz: tuple[float, float]
) -> tuple[Path, Path]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    samples = pcm.astype(np.float64) / 32768.0
    spectrogram_path = path.with_name(f"{path.stem}_spectrogram.png")
    fft_path = path.with_name(f"{path.stem}_fft.png")

    figure, axis = plt.subplots(figsize=(10, 5))
    axis.specgram(samples, NFFT=2048, Fs=PCM_RATE_HZ, noverlap=1536, cmap="magma")
    for tone in tones_hz:
        axis.axhline(tone, color="cyan", linewidth=0.8, linestyle="--")
    axis.set(title="Captured microphone spectrogram", xlabel="time (s)", ylabel="frequency (Hz)")
    axis.set_ylim(0, PCM_RATE_HZ / 2)
    figure.tight_layout()
    figure.savefig(spectrogram_path, dpi=150)
    plt.close(figure)

    window = np.hanning(len(samples))
    magnitude_db = 20.0 * np.log10(
        np.maximum(np.abs(np.fft.rfft(samples * window)), np.finfo(float).tiny)
    )
    frequencies = np.fft.rfftfreq(len(samples), 1.0 / PCM_RATE_HZ)
    figure, axis = plt.subplots(figsize=(10, 5))
    axis.plot(frequencies, magnitude_db, linewidth=0.7)
    for tone in tones_hz:
        axis.axvline(tone, color="red", linewidth=0.8, linestyle="--", label=f"{tone:.0f} Hz")
    axis.set(title="Full-capture Hann-windowed FFT", xlabel="frequency (Hz)", ylabel="magnitude (dBFS, relative)")
    axis.set_xlim(0, PCM_RATE_HZ / 2)
    axis.grid(True, alpha=0.25)
    axis.legend()
    figure.tight_layout()
    figure.savefig(fft_path, dpi=150)
    plt.close(figure)
    return spectrogram_path, fft_path


def report_capture(
    path: Path,
    parsed: ParsedCapture,
    tones_hz: tuple[float, float],
    plot: bool,
) -> tuple[TimingReport, list[tuple[float, float, float]]]:
    timing = analyze_timing(parsed)
    duration_s = len(parsed.pcm) / PCM_RATE_HZ
    print(
        f"audio: {len(parsed.pcm)} samples in {len(parsed.chunks)} chunks "
        f"({duration_s:.3f} s received)"
    )
    print(
        f"AUD timestamp discontinuities: {len(timing.gaps)}; "
        f"inferred missing samples={timing.inferred_missing_samples}; "
        f"capture coverage={timing.coverage_percent:.3f}%"
    )
    for index, error_ms, missing in timing.gaps:
        print(
            f"  chunk {index}: timestamp interval error={error_ms:+.3f} ms, "
            f"inferred missing={missing} samples"
        )
    print(
        f"effective sample rate on continuous intervals: {timing.effective_rate_hz:.3f} Hz; "
        f"drift vs 48000 Hz={timing.drift_ppm:+.1f} ppm"
    )
    print(
        f"IMU during audio: {timing.imu_count} lines, sustained rate={timing.imu_rate_hz:.2f} Hz "
        "(target near 219 Hz)"
    )
    if parsed.malformed_headers or parsed.truncated_payloads:
        print(
            f"framing warnings: malformed AUD headers={parsed.malformed_headers}, "
            f"truncated payloads={parsed.truncated_payloads}"
        )
    snrs = tone_snrs(parsed.pcm, tones_hz)
    for requested, detected, snr_db in snrs:
        print(
            f"tone {requested:.0f} Hz: detected bin={detected:.2f} Hz, "
            f"bin/local-noise SNR={snr_db:.2f} dB"
        )
    if plot:
        spectrogram_path, fft_path = save_plots(path, parsed.pcm, tones_hz)
        print(f"spectrogram: {spectrogram_path}")
        print(f"FFT: {fft_path}")
    return timing, snrs


def synthetic_pcm(
    duration_s: float, tones_hz: tuple[float, float], truth_snr_db: float
) -> tuple[np.ndarray, float]:
    sample_count = int(round(duration_s * PCM_RATE_HZ))
    indices = np.arange(sample_count, dtype=np.float64)
    amplitude = 0.05
    window = np.hanning(sample_count)
    desired_ratio = 10.0 ** (truth_snr_db / 10.0)
    noise_sigma = amplitude * np.sum(window) / (
        2.0 * np.sqrt(desired_ratio * np.sum(window * window))
    )
    rng = np.random.default_rng(20260830)
    signal = sum(
        amplitude * np.sin(2.0 * np.pi * tone * indices / PCM_RATE_HZ)
        for tone in tones_hz
    )
    samples = np.clip(signal + rng.normal(0.0, noise_sigma, sample_count), -1.0, 1.0)
    return np.rint(samples * 32767.0).astype("<i2"), noise_sigma


def write_synthetic_stream(
    path: Path, pcm: np.ndarray, dropped_chunk: int | None
) -> int:
    chunk_samples = 960
    start_us = 1_000_000
    imu_times = start_us + np.rint(
        np.arange(0.0, len(pcm) / PCM_RATE_HZ, 1.0 / 219.0) * 1e6
    ).astype(np.int64)
    imu_index = 0
    written_samples = 0
    with path.open("wb") as sink:
        sink.write(b"# synthetic TABLE-ACOUSTIC capture\n")
        sink.write(b"CFG,gyro_range_dps=2048,accel_range_g=16,odr_hz=219\n")
        for chunk_index, offset in enumerate(range(0, len(pcm), chunk_samples)):
            end = min(offset + chunk_samples, len(pcm))
            completion_us = start_us + round(end / PCM_RATE_HZ * 1e6)
            while imu_index < len(imu_times) and imu_times[imu_index] <= completion_us:
                t_us = imu_times[imu_index]
                sink.write(f"IMU,{t_us},0,0,0,0,0,-1\n".encode())
                imu_index += 1
            if chunk_index == dropped_chunk:
                continue
            payload = pcm[offset:end].tobytes()
            sink.write(f"AUD,{completion_us},{len(payload)}\n".encode())
            sink.write(payload)
            written_samples += end - offset
    return written_samples


def selftest() -> None:
    tones = (DEFAULT_LEFT_HZ, DEFAULT_RIGHT_HZ)
    truth_snr_db = 40.0
    pcm, noise_sigma = synthetic_pcm(2.0, tones, truth_snr_db)
    with tempfile.TemporaryDirectory(prefix="acoustic_probe_") as temporary:
        full_path = Path(temporary) / "synthetic_full.bin"
        gap_path = Path(temporary) / "synthetic_gap.bin"
        full_written = write_synthetic_stream(full_path, pcm, dropped_chunk=None)
        gap_written = write_synthetic_stream(gap_path, pcm, dropped_chunk=42)
        full = parse_capture(full_path)
        gap = parse_capture(gap_path)
        full_timing = analyze_timing(full)
        gap_timing = analyze_timing(gap)
        snrs = tone_snrs(full.pcm, tones)

        exact_pcm = len(full.pcm) == full_written == len(pcm) and np.array_equal(full.pcm, pcm)
        snr_errors = [abs(measured - truth_snr_db) for _, _, measured in snrs]
        gap_ok = (
            len(gap_timing.gaps) == 1
            and gap_timing.inferred_missing_samples == 960
            and len(gap.pcm) == gap_written == len(pcm) - 960
        )
        print(
            f"synthetic signal: {len(pcm)} samples, tones={tones[0]:.0f}/{tones[1]:.0f} Hz, "
            f"noise sigma={noise_sigma:.6f}, constructed bin SNR={truth_snr_db:.2f} dB"
        )
        print(
            f"full-stream PCM: parsed={len(full.pcm)}, expected={len(pcm)}, "
            f"sample-for-sample match={exact_pcm}, loss={len(pcm) - len(full.pcm)}"
        )
        print(
            f"full-stream timing: gaps={len(full_timing.gaps)}, "
            f"effective rate={full_timing.effective_rate_hz:.3f} Hz, "
            f"IMU rate={full_timing.imu_rate_hz:.2f} Hz"
        )
        print(
            f"dropped chunk 42: parsed={len(gap.pcm)}, gaps={len(gap_timing.gaps)}, "
            f"inferred missing={gap_timing.inferred_missing_samples}, detection={gap_ok}"
        )
        for requested, detected, measured in snrs:
            print(
                f"SNR {requested:.0f} Hz (bin {detected:.2f} Hz): measured={measured:.2f} dB, "
                f"truth={truth_snr_db:.2f} dB, error={measured - truth_snr_db:+.2f} dB"
            )
        if not exact_pcm:
            raise SystemExit("selftest failed: PCM did not round-trip exactly")
        if full_timing.gaps:
            raise SystemExit("selftest failed: false timestamp discontinuity")
        if not gap_ok:
            raise SystemExit("selftest failed: dropped chunk was not detected correctly")
        if any(error > 3.0 for error in snr_errors):
            raise SystemExit("selftest failed: measured SNR differs from truth by more than 3 dB")
    print("SELFTEST PASS")


def main() -> None:
    sys.stdout.reconfigure(line_buffering=True)
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--emit", action="store_true")
    mode.add_argument("--sweep", action="store_true")
    mode.add_argument("--parse", metavar="FILE", type=Path)
    mode.add_argument("--capture", metavar="FILE", type=Path)
    mode.add_argument("--selftest", action="store_true")
    parser.add_argument("--f-left", type=float, help=f"left tone Hz (default {DEFAULT_LEFT_HZ:.0f})")
    parser.add_argument("--f-right", type=float, help=f"right tone Hz (default {DEFAULT_RIGHT_HZ:.0f})")
    parser.add_argument("--amp", type=float, default=DEFAULT_AMP)
    parser.add_argument("--plot", action="store_true")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=0.0, help="capture duration; 0 means until Ctrl-C")
    args = parser.parse_args()

    if not 0.0 <= args.amp <= 1.0:
        parser.error("--amp must be between 0 and 1")
    if args.emit:
        emit(args.f_left, args.f_right, args.amp)
    elif args.sweep:
        sweep(args.amp)
    elif args.parse:
        tones = (
            DEFAULT_LEFT_HZ if args.f_left is None else args.f_left,
            DEFAULT_RIGHT_HZ if args.f_right is None else args.f_right,
        )
        parsed = parse_capture(args.parse)
        if not parsed.chunks:
            raise SystemExit(f"{args.parse}: no complete AUD chunks found")
        report_capture(args.parse, parsed, tones, args.plot)
    elif args.capture:
        capture(args.port, args.capture, args.baud, args.seconds)
    else:
        selftest()


if __name__ == "__main__":
    main()
