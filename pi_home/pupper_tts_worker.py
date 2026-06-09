#!/usr/bin/env python3
"""Standalone TTS worker for Pupper bridge.

Called as subprocess: python3 pupper_tts_worker.py "text to speak"
Synthesizes via DashScope Qwen TTS, saves WAV, plays via paplay.
Prints "OK <pcm_bytes>" on success, exits 1 on failure.
"""
import sys
import os
import base64
import threading
import tempfile
import wave
import subprocess

import dashscope
from dashscope.audio.qwen_tts_realtime import (
    AudioFormat,
    QwenTtsRealtime,
    QwenTtsRealtimeCallback,
)

API_KEY = os.environ.get("DASHSCOPE_API_KEY", "")
WS_URL = os.environ.get(
    "OPENTALKING_QWEN_TTS_WS_URL",
    "wss://dashscope-intl.aliyuncs.com/api-ws/v1/realtime",
)
VOICE = os.environ.get("OPENTALKING_TTS_VOICE", "Kai")
# qwen3-tts-instruct-flash-realtime is the model that supports BOTH built-in
# voices (Kai / Cherry / Ethan / etc.) AND custom `instructions`. It is faster
# than the non-flash variant.
#
# Why NOT qwen3-tts-vc-realtime: that model is the voice-cloning variant and
# *only* accepts custom-cloned voice IDs (qwen-tts-vc-*). All built-in voices
# are rejected with CLIENT_ERROR there. We had been on it accidentally with
# `qwen-tts-vc-twgirl-voice-...` (a 24-hr cloned female voice) which was the
# regression pupper users noticed as "voice changed to female". 2026-05-31:
# restored to instruct-flash + Kai (the original male voice).
MODEL = "qwen3-tts-instruct-flash-realtime"
INSTRUCTIONS = (
    "請模仿一隻機器狗說話：聲音略帶機械感、節奏穩定，"
    "但偶有一點點俏皮的上揚，整體保持冷靜可靠。"
)


def synthesize(text: str) -> bytes:
    dashscope.api_key = API_KEY
    pcm = bytearray()
    done = threading.Event()
    errors = []

    class Cb(QwenTtsRealtimeCallback):
        def on_event(self, m):
            if not isinstance(m, dict):
                return
            t = m.get("type", "")
            if t == "response.audio.delta":
                pcm.extend(base64.b64decode(m["delta"]))
            elif t in ("response.done", "session.finished"):
                done.set()
            elif "error" in t or "error" in m:
                errors.append(str(m))
                done.set()

        def on_error(self, *a, **kw):
            errors.append(f"on_error: {a}")
            done.set()

        def on_close(self, *a, **kw):
            done.set()

    client = QwenTtsRealtime(model=MODEL, callback=Cb(), url=WS_URL)
    client.connect()
    client.update_session(
        voice=VOICE,
        response_format=AudioFormat.PCM_24000HZ_MONO_16BIT,
        mode="commit",
        sample_rate=24000,
        language_type="Chinese",
        instructions=INSTRUCTIONS,
        optimize_instructions=False,
    )
    client.append_text(text)
    client.commit()

    if not done.wait(timeout=20.0):
        raise RuntimeError("TTS synthesis timeout")

    try:
        client.finish()
    except Exception:
        pass
    try:
        client.close()
    except Exception:
        pass

    if errors:
        raise RuntimeError(f"TTS errors: {errors}")
    return bytes(pcm)


def main():
    if len(sys.argv) < 2 or not sys.argv[1].strip():
        print("ERR no text", file=sys.stderr)
        sys.exit(1)

    text = sys.argv[1]
    try:
        pcm = synthesize(text)
    except Exception as e:
        print(f"ERR {e}", file=sys.stderr)
        sys.exit(1)

    # Write WAV and play
    tmp = tempfile.NamedTemporaryFile(suffix=".wav", delete=False)
    with wave.open(tmp.name, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(24000)
        w.writeframes(pcm)

    subprocess.Popen(["paplay", tmp.name])
    print(f"OK {len(pcm)}")


if __name__ == "__main__":
    main()
