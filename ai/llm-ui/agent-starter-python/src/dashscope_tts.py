"""DashScope (Qwen) Realtime TTS plugin for LiveKit Agents.

Drop-in replacement for ``livekit.plugins.cartesia.TTS`` that uses
Alibaba DashScope's Qwen Realtime TTS over WebSocket.

Usage:
    from dashscope_tts import TTS as DashScopeTTS
    session = AgentSession(
        ...,
        tts=DashScopeTTS(voice="Cherry", model="qwen3-tts-flash-realtime"),
    )

Requires:
    pip install 'dashscope>=1.25.11'

Env vars:
    DASHSCOPE_API_KEY                          (required)
    OPENTALKING_QWEN_TTS_WS_URL                (optional, defaults to international endpoint)
    OPENTALKING_QWEN_TTS_LANGUAGE              (optional, defaults to "Chinese")
    OPENTALKING_QWEN_TTS_INSTRUCTIONS          (optional, free-text style prompt)
    OPENTALKING_QWEN_TTS_OPTIMIZE_INSTRUCTIONS (optional, "true"/"false", default false)
    OPENTALKING_QWEN_TTS_SPEECH_RATE           (optional, 0.5-2.0, default unset)
    OPENTALKING_QWEN_TTS_PITCH_RATE            (optional, 0.5-2.0, default unset)
    OPENTALKING_QWEN_TTS_VOLUME                (optional, 0-100, default unset)
"""

from __future__ import annotations

import asyncio
import base64
import logging
import os
import re
from typing import Any

from livekit.agents import APIConnectionError, APIStatusError, tts, utils
from livekit.agents.types import APIConnectOptions, DEFAULT_API_CONNECT_OPTIONS

logger = logging.getLogger(__name__)

# Strip Cartesia-style SSML tags that may leak through from older prompts.
# Example: <emotion value="excited" /> Hello!  ->  Hello!
_TAG_RE = re.compile(r"<\s*/?\s*[a-zA-Z][\w-]*[^>]*>")


def _clean_text(text: str) -> str:
    """Remove XML/SSML tags and collapse the resulting whitespace.

    DashScope speaks input verbatim, so any unstripped <emotion .../> will be
    read out loud. We always strip these as defense-in-depth even if the prompt
    no longer asks for them.
    """
    cleaned = _TAG_RE.sub(" ", text)
    cleaned = re.sub(r"\s+", " ", cleaned).strip()
    return cleaned

# DashScope realtime TTS always speaks PCM 24kHz mono 16-bit on the wire.
WIRE_SR = 24000

DEFAULT_MODEL = "qwen3-tts-flash-realtime"
# Voice-cloned ("vc") voices need a different realtime model. Override via env
# OPENTALKING_QWEN_TTS_MODEL if a newer VC model is released.
DEFAULT_VC_MODEL = "qwen3-tts-vc-realtime-2026-01-15"
DEFAULT_VOICE = "Cherry"  # built-in voice, no clone required
DEFAULT_WS_URL = "wss://dashscope-intl.aliyuncs.com/api-ws/v1/realtime"
DEFAULT_LANGUAGE = "Chinese"
# Persona prompt for the Pupster robot dog. Picked from the r03 A/B winner:
# voice clones in qwen3-tts-vc-realtime barely respond to "excited/sad" style
# nudges, but DO respond to "robotic/measured" cues — which actually fits the
# Jarvis robot-dog persona perfectly.
DEFAULT_INSTRUCTIONS = (
    "請模仿一隻機器狗說話：聲音略帶機械感、節奏穩定，"
    "但偶有一點點俏皮的上揚，整體保持冷靜可靠。"
)


def _env_or(key: str, default: str) -> str:
    raw = os.environ.get(key)
    return default if raw is None else raw  # explicit "" → empty (disable)


def _env_bool(key: str, default: bool) -> bool:
    raw = os.environ.get(key, "").strip().lower()
    if raw in ("1", "true", "yes", "on"):
        return True
    if raw in ("0", "false", "no", "off"):
        return False
    return default


def _env_float(key: str) -> float | None:
    raw = os.environ.get(key, "").strip()
    if not raw:
        return None
    try:
        return float(raw)
    except ValueError:
        logger.warning("invalid %s=%r — ignored", key, raw)
        return None


def _env_int(key: str) -> int | None:
    raw = os.environ.get(key, "").strip()
    if not raw:
        return None
    try:
        return int(raw)
    except ValueError:
        logger.warning("invalid %s=%r — ignored", key, raw)
        return None


def _resolve_model(voice: str, model_override: str | None) -> str:
    """Pick the right realtime model based on the voice id.

    - Built-in voices (Cherry, Ethan, ...) → ``qwen3-tts-flash-realtime``
    - Custom voice clones (id starts with ``qwen-tts-vc-``) →
      ``qwen3-tts-vc-realtime-2026-01-15``
    - Any explicit ``model_override`` (constructor or env) wins.
    """
    if model_override:
        return model_override
    if voice.startswith("qwen-tts-vc-"):
        return DEFAULT_VC_MODEL
    return DEFAULT_MODEL


class TTS(tts.TTS):
    def __init__(
        self,
        *,
        voice: str = DEFAULT_VOICE,
        model: str | None = None,
        api_key: str | None = None,
        ws_url: str | None = None,
        language: str | None = None,
        instructions: str | None = None,
        optimize_instructions: bool | None = None,
        speech_rate: float | None = None,
        pitch_rate: float | None = None,
        volume: int | None = None,
        chunk_timeout_s: float = 20.0,
    ) -> None:
        super().__init__(
            capabilities=tts.TTSCapabilities(streaming=False, aligned_transcript=False),
            sample_rate=WIRE_SR,
            num_channels=1,
        )
        self._api_key = api_key or os.environ.get("DASHSCOPE_API_KEY", "").strip()
        if not self._api_key:
            raise ValueError(
                "DASHSCOPE_API_KEY must be set (env var or api_key kwarg)"
            )

        self._voice = voice
        model_override = model or os.environ.get("OPENTALKING_QWEN_TTS_MODEL", "").strip() or None
        self._model = _resolve_model(voice, model_override)
        self._ws_url = (
            ws_url
            or os.environ.get("OPENTALKING_QWEN_TTS_WS_URL", "").strip()
            or DEFAULT_WS_URL
        )
        self._language = (
            language
            or os.environ.get("OPENTALKING_QWEN_TTS_LANGUAGE", "").strip()
            or DEFAULT_LANGUAGE
        )

        # Expressive controls — env overrides ctor, ctor overrides default.
        # Setting OPENTALKING_QWEN_TTS_INSTRUCTIONS="" explicitly disables.
        if instructions is None:
            instr = _env_or("OPENTALKING_QWEN_TTS_INSTRUCTIONS", DEFAULT_INSTRUCTIONS)
        else:
            instr = instructions
        self._instructions: str | None = instr.strip() if instr and instr.strip() else None

        self._optimize_instructions = (
            optimize_instructions
            if optimize_instructions is not None
            else _env_bool("OPENTALKING_QWEN_TTS_OPTIMIZE_INSTRUCTIONS", False)
        )
        self._speech_rate = (
            speech_rate
            if speech_rate is not None
            else _env_float("OPENTALKING_QWEN_TTS_SPEECH_RATE")
        )
        self._pitch_rate = (
            pitch_rate
            if pitch_rate is not None
            else _env_float("OPENTALKING_QWEN_TTS_PITCH_RATE")
        )
        self._volume = (
            volume
            if volume is not None
            else _env_int("OPENTALKING_QWEN_TTS_VOLUME")
        )
        self._timeout_s = chunk_timeout_s

        logger.info(
            "DashScope TTS: voice=%s model=%s instructions=%r optimize=%s "
            "speech_rate=%s pitch_rate=%s volume=%s",
            self._voice,
            self._model,
            (self._instructions or "")[:60],
            self._optimize_instructions,
            self._speech_rate,
            self._pitch_rate,
            self._volume,
        )

    @property
    def model(self) -> str:
        return self._model

    @property
    def provider(self) -> str:
        return "DashScope"

    def synthesize(
        self,
        text: str,
        *,
        conn_options: APIConnectOptions = DEFAULT_API_CONNECT_OPTIONS,
    ) -> "ChunkedStream":
        return ChunkedStream(tts=self, input_text=text, conn_options=conn_options)


class ChunkedStream(tts.ChunkedStream):
    def __init__(
        self,
        *,
        tts: TTS,
        input_text: str,
        conn_options: APIConnectOptions,
    ) -> None:
        super().__init__(tts=tts, input_text=input_text, conn_options=conn_options)
        self._tts: TTS = tts

    async def _run(self, output_emitter: tts.AudioEmitter) -> None:
        try:
            import dashscope
            from dashscope.audio.qwen_tts_realtime import (
                AudioFormat,
                QwenTtsRealtime,
                QwenTtsRealtimeCallback,
            )
        except ImportError as e:
            raise APIConnectionError(
                "dashscope package required: pip install 'dashscope>=1.25.11'"
            ) from e

        dashscope.api_key = self._tts._api_key
        loop = asyncio.get_running_loop()
        inbox: asyncio.Queue[Any] = asyncio.Queue()

        class _Cb(QwenTtsRealtimeCallback):
            def on_event(self, message: Any) -> None:
                if isinstance(message, dict):
                    try:
                        loop.call_soon_threadsafe(inbox.put_nowait, message)
                    except Exception:
                        logger.exception("DashScope TTS inbox push failed")

            def on_error(self, *args: Any, **kwargs: Any) -> None:
                msg = " ".join(str(a) for a in args) + " " + " ".join(
                    f"{k}={v}" for k, v in kwargs.items()
                )
                try:
                    loop.call_soon_threadsafe(
                        inbox.put_nowait,
                        {"type": "error", "error": msg.strip() or "unknown error"},
                    )
                except Exception:
                    pass

            def on_close(self, *args: Any, **kwargs: Any) -> None:
                try:
                    loop.call_soon_threadsafe(inbox.put_nowait, {"type": "closed"})
                except Exception:
                    pass

        cb = _Cb()
        client = QwenTtsRealtime(
            model=self._tts._model, callback=cb, url=self._tts._ws_url
        )

        text_to_speak = _clean_text(self._input_text)
        # Always initialize the emitter first (LiveKit expects this even if we emit
        # zero audio for empty inputs).
        output_emitter.initialize(
            request_id=utils.shortuuid(),
            sample_rate=WIRE_SR,
            num_channels=1,
            mime_type="audio/pcm",
        )
        if not text_to_speak:
            output_emitter.flush()
            return

        def _connect_and_send() -> None:
            client.connect()
            session_kwargs: dict[str, Any] = {
                "voice": self._tts._voice,
                "response_format": AudioFormat.PCM_24000HZ_MONO_16BIT,
                "mode": "commit",
                "sample_rate": WIRE_SR,
                "language_type": self._tts._language,
            }
            if self._tts._instructions:
                session_kwargs["instructions"] = self._tts._instructions
                session_kwargs["optimize_instructions"] = self._tts._optimize_instructions
            if self._tts._speech_rate is not None:
                session_kwargs["speech_rate"] = self._tts._speech_rate
            if self._tts._pitch_rate is not None:
                session_kwargs["pitch_rate"] = self._tts._pitch_rate
            if self._tts._volume is not None:
                session_kwargs["volume"] = self._tts._volume
            client.update_session(**session_kwargs)
            client.append_text(text_to_speak)
            client.commit()

        def _close() -> None:
            try:
                client.finish()
            except Exception:
                pass
            try:
                client.close()
            except Exception:
                pass

        try:
            await asyncio.to_thread(_connect_and_send)
            while True:
                try:
                    msg = await asyncio.wait_for(inbox.get(), timeout=self._tts._timeout_s)
                except asyncio.TimeoutError as e:
                    raise APIConnectionError(
                        f"DashScope TTS timed out after {self._tts._timeout_s}s"
                    ) from e

                mtype = msg.get("type")
                if mtype == "response.audio.delta":
                    raw = base64.b64decode(msg["delta"])
                    output_emitter.push(raw)
                elif mtype in ("response.done", "session.finished"):
                    break
                elif mtype == "error":
                    raise APIStatusError(
                        message=str(msg.get("error", msg)),
                        status_code=500,
                        request_id=None,
                        body=None,
                    )
                elif mtype == "closed":
                    break

            output_emitter.flush()
        finally:
            await asyncio.to_thread(_close)
