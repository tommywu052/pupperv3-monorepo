#!/usr/bin/env python3
"""
Pupper TTS Worker - CosyVoice (Thor) HTTP synthesis + paplay playback.

Drop-in alternative to ``pupper_tts_worker.py`` (which uses DashScope WSS).
Bridge dispatches via ``_TTS_WORKERS`` map in pupper_bridge.py.

Usage:
    python3 pupper_tts_worker_cosyvoice.py "要說的話" [spk_id]

Output (parsed by bridge, MUST be ``OK <bytes>`` on success):
    OK 364800
exits 0 on success, prints ``ERR <msg>`` to stderr and exits 1 on failure.

Environment variables:
    COSYVOICE_TTS_URL              default http://192.168.31.52:50000/tts
    COSYVOICE_SPK_ID               default "default"   (overridden by argv[2])
    COSYVOICE_TARGET_SR            default 24000
    COSYVOICE_TIMEOUT_S            default 30
    COSYVOICE_GAIN_MODE            default "rms"    ("rms" | "peak" | "off")
    COSYVOICE_TARGET_RMS_DBFS      default -10.0
    COSYVOICE_SOFTCLIP             default 1
    COSYVOICE_MAKEUP_DB            default 6.0

Loudness defaults are copied from duckmini's cosyvoice worker. Pupper's
speaker is different hardware (USB DAC / 3.5mm via paplay rather than
MAX98357A via aplay) so the same digital headroom math may sound louder
or quieter — tune via env once you can listen.
"""

import io
import json
import os
import subprocess
import sys
import tempfile
import urllib.request
import wave

_no_proxy_opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

TTS_URL = os.environ.get("COSYVOICE_TTS_URL", "http://192.168.31.52:50000/tts")
DEFAULT_SPK_ID = os.environ.get("COSYVOICE_SPK_ID", "default")
TARGET_SR = int(os.environ.get("COSYVOICE_TARGET_SR", "24000"))
TIMEOUT_S = float(os.environ.get("COSYVOICE_TIMEOUT_S", "30"))


def synthesize(text: str, spk_id: str):
    payload = json.dumps({
        "text": text,
        "spk_id": spk_id,
        "target_sr": TARGET_SR,
    }).encode("utf-8")
    req = urllib.request.Request(
        TTS_URL,
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with _no_proxy_opener.open(req, timeout=TIMEOUT_S) as resp:
        wav_bytes = resp.read()

    if not wav_bytes or wav_bytes[:4] != b"RIFF":
        raise RuntimeError(f"CosyVoice did not return WAV ({len(wav_bytes)}B)")

    with wave.open(io.BytesIO(wav_bytes), "rb") as w:
        sr = w.getframerate()
        ch = w.getnchannels()
        sw = w.getsampwidth()
        pcm = w.readframes(w.getnframes())
    if sw != 2 or ch != 1:
        raise RuntimeError(f"Unexpected WAV format: ch={ch} sw={sw}")
    return pcm, sr


def amplify_pcm_cosy(pcm_data: bytes) -> bytes:
    """RMS-target + tanh soft-clip. Falls back to passthrough if numpy missing."""
    try:
        import numpy as np
    except ImportError:
        return pcm_data

    samples = np.frombuffer(pcm_data, dtype=np.int16)
    if samples.size == 0:
        return pcm_data

    mode = os.environ.get("COSYVOICE_GAIN_MODE", "rms").lower()
    if mode == "off":
        return pcm_data

    fs = 32767.0
    f = samples.astype(np.float32)

    if mode == "peak":
        gain_db = float(os.environ.get("TTS_GAIN_DB", "6.0"))
        normalize = os.environ.get("TTS_NORMALIZE", "1") != "0"
        peak = float(np.max(np.abs(f)))
        if normalize and peak > 0:
            target = fs * (10.0 ** (-0.5 / 20.0))
            f = f * (target / peak)
        if gain_db != 0.0:
            f = f * (10.0 ** (gain_db / 20.0))
        np.clip(f, -fs - 1, fs, out=f)
    else:
        target_rms_dbfs = float(os.environ.get("COSYVOICE_TARGET_RMS_DBFS", "-10.0"))
        makeup_db = float(os.environ.get("COSYVOICE_MAKEUP_DB", "6.0"))
        softclip = os.environ.get("COSYVOICE_SOFTCLIP", "1") != "0"
        rms = float(np.sqrt(np.mean(f * f)))
        if rms > 1.0:
            target_rms = fs * (10.0 ** (target_rms_dbfs / 20.0))
            f = f * (target_rms / rms)
        if makeup_db:
            f = f * (10.0 ** (makeup_db / 20.0))
        if softclip:
            np.divide(f, fs, out=f)
            np.tanh(f, out=f)
            np.multiply(f, fs * 0.98, out=f)
        else:
            np.clip(f, -fs - 1, fs, out=f)

    return f.astype(np.int16).tobytes()


def play_pcm(pcm_data: bytes, sample_rate: int) -> int:
    try:
        pcm_data = amplify_pcm_cosy(pcm_data)
    except Exception as e:
        print(f"WARN amplify failed: {e}", file=sys.stderr)

    tmp = tempfile.NamedTemporaryFile(suffix=".wav", delete=False)
    try:
        with wave.open(tmp.name, "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(sample_rate)
            w.writeframes(pcm_data)
        subprocess.Popen(["paplay", tmp.name])
    finally:
        # paplay reads asynchronously; leave the temp file in /tmp,
        # OS will recycle it. Same pattern as dashscope worker.
        pass
    return len(pcm_data)


def main():
    if len(sys.argv) < 2 or not sys.argv[1].strip():
        print("ERR no text", file=sys.stderr)
        sys.exit(1)

    text = sys.argv[1]
    spk_id = sys.argv[2] if len(sys.argv) >= 3 and sys.argv[2].strip() else DEFAULT_SPK_ID

    try:
        pcm, sr = synthesize(text, spk_id)
    except Exception as e:
        print(f"ERR cosyvoice synth failed: {e}", file=sys.stderr)
        sys.exit(1)

    if not pcm:
        print("ERR cosyvoice produced no audio", file=sys.stderr)
        sys.exit(1)

    n = play_pcm(pcm, sr)
    print(f"OK {n}")


if __name__ == "__main__":
    main()
