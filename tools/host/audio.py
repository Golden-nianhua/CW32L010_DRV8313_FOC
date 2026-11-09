"""Host-side decoding and paced binary motor audio streaming."""
from dataclasses import asdict
from math import isfinite
import time
import struct
import subprocess
import os


class AudioWriter:
    """Aligned producer for the pinned pyOCD RTT channel implementation."""

    def __init__(self, channel):
        self.target = channel._target
        self.offsets = channel._offsets_addr
        self.buffer = channel._buffer_address
        self.size = channel.size
        self.offset = self.target.read32(self.offsets)

    def _store(self, address, data):
        aligned = len(data) & ~3
        if aligned:
            words = struct.unpack(f"<{aligned // 4}I", data[:aligned])
            self.target.write_memory_block32(address, words)
        if aligned != len(data):
            self.target.write_memory_block8(address + aligned, data[aligned:])

    def write(self, data):
        read = self.target.read32(self.offsets + 4)
        if read >= self.size:
            raise RuntimeError("invalid PCM read offset")
        free = (read - self.offset - 1) & (self.size - 1)
        count = min(free, len(data))
        if count < len(data):
            count &= ~3
        if not count:
            return 0
        first = min(count, self.size - self.offset)
        self._store(self.buffer + self.offset, data[:first])
        self._store(self.buffer, data[first:count])
        self.offset = (self.offset + count) & (self.size - 1)
        self.target.write32(self.offsets, self.offset)
        return count


def play_file(link, args):
    import numpy as np
    from imageio_ffmpeg import get_ffmpeg_exe
    from scipy.signal import bilinear, butter, lfilter, sosfilt

    if link.audio_down is None:
        raise RuntimeError("firmware PCM channel is unavailable")
    if (not isfinite(args.start) or args.start < 0 or abs(args.speed) > 30000 or
            not 0 < args.amplitude <= 6000 or not isfinite(args.preemphasis_us) or
            args.preemphasis_us < 0 or not isfinite(args.bass_db) or not -12 <= args.bass_db <= 18):
        raise ValueError("play requires start >= 0, speed within 30 rad/s and amplitude within 6 V and preemphasis >= 0")
    path = args.file if args.file.is_absolute() else link.directory / args.file
    command = [get_ffmpeg_exe(), "-nostdin", "-hide_banner", "-loglevel", "error",
               "-ss", str(args.start), "-i", str(path), "-vn", "-ac", "1", "-ar", "48000"]
    if args.seconds is not None:
        command.extend(("-t", str(args.seconds)))
    command.extend(("-f", "f32le", "-acodec", "pcm_f32le", "pipe:1"))
    decoded = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                             creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
    if decoded.returncode:
        raise ValueError(decoded.stderr.decode("utf-8", errors="replace").strip())
    mono = np.frombuffer(decoded.stdout, dtype="<f4").astype(np.float64)
    if len(mono) == 0:
        raise ValueError("selected audio segment is empty")
    mono = sosfilt(butter(2, 60, "highpass", fs=48000, output="sos"), mono)
    if args.bass_db:
        shelf = 10 ** (args.bass_db / 40)
        corner = 2 * np.pi * 300
        b, a = bilinear([1, corner * shelf], [1, corner / shelf], fs=48000)
        mono = lfilter(b, a, mono)
    if args.preemphasis_us:
        b, a = bilinear([args.preemphasis_us * 1e-6, 1], [1 / (2 * np.pi * 12000), 1], fs=48000)
        mono = lfilter(b, a, mono)
    peak = float(np.max(np.abs(mono)))
    if peak:
        mono *= 0.95 / peak
    pcm = np.round(np.clip(mono, -1, 1) * 32767).astype("<i2")
    frames = len(pcm)
    if args.transport == "mulaw":
        linear = pcm.astype(np.int32)
        sign = (linear < 0).astype(np.int32) << 7
        magnitude = np.minimum(np.abs(linear), 32635) + 132
        exponent = np.clip(np.floor(np.log2(magnitude)).astype(np.int32) - 7, 0, 7)
        mantissa = (magnitude >> (exponent + 3)) & 15
        payload = (~(sign | (exponent << 4) | mantissa)).astype(np.uint8).tobytes()
    else:
        payload = pcm.tobytes()
    result = {"file": str(path.resolve()), "decoder": "ffmpeg", "sample_rate": 48000, "frames": frames,
              "speed_rad_s": args.speed / 1000, "peak_mv": args.amplitude,
              "transport": args.transport, "bytes": len(payload), "preemphasis_us": args.preemphasis_us, "bass_db": args.bass_db}
    try:
        link.command("X")
        link.command("C", 600)
        link.wait_for(lambda s: s.state == 3, "motor calibration did not finish")
        link.command("S", args.speed)
        link.command("E")
        list(link.collect(2.0))
        before = link.command("M", args.amplitude, frames, int(args.transport == "mulaw"))
        writer = AudioWriter(link.audio_down)
        offset = 0
        started = time.monotonic()
        deadline = started + frames / 48000 * 1.25 + 10
        poll_at = started
        while time.monotonic() < deadline:
            sent = 0
            if offset < len(payload):
                sent = writer.write(payload[offset:offset + 384])
                offset += sent
            now = time.monotonic()
            if now >= poll_at:
                link.read()
                poll_at = now + .1
                if link.latest.state != 4 or link.latest.event in (4, 5):
                    raise RuntimeError(f"audio interrupted: {link.latest.text()}")
                if link.latest.audio_frames >= frames:
                    result.update(seconds=round(now - started, 3), effective_rate=round(frames / (now - started), 1), status=asdict(link.latest),
                                  missed_samples=link.latest.missed_samples - before.missed_samples,
                                  encoder_errors=link.latest.encoder_errors - before.encoder_errors)
                    warnings = []
                    if link.latest.audio_underruns:
                        warnings.append(f"audio stream starved {link.latest.audio_underruns} times")
                    expected_seconds = frames / 48000
                    if expected_seconds >= 2 and abs(now - started - expected_seconds) > max(.15, expected_seconds * .02):
                        warnings.append(f"{expected_seconds:.3f} s of audio took {now - started:.3f} s")
                    result.update(ok=not warnings, warnings=warnings)
                    return result
            if not sent:
                time.sleep(.0002)
        raise TimeoutError(f"PCM playback timed out at {link.latest.audio_frames}/{frames} samples")
    finally:
        stopped = link.command("X")
        result["final_stop"] = asdict(stopped)
        if stopped.state not in (0, 3):
            raise RuntimeError("final motor stop could not be confirmed")
