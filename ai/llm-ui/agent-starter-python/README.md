# Pupster / Jarvis Voice Agent (Pupper v3)

LiveKit-based voice AI for the Pupper v3 robot on Raspberry Pi 5.  
基於 LiveKit 的 Pupper v3 語音 AI（Jarvis / Pupster），部署於 Raspberry Pi 5。

This package replaces the upstream LiveKit starter template with a **production on-robot stack**: wake word, pre-warm gating, OpenAI Realtime LLM, DashScope Qwen TTS, and ROS 2 tool calls for locomotion, animations, and vision.

---

## Architecture / 系統架構

```
  USB mic (BOYALINK)
       │
       ▼
  pupster-wake.service          OpenWakeWord ("Hey Jarvis")
       │  touch /tmp/pupster_gate
       ▼
  llm-agent.service             LiveKit Agents (this package)
       │  _GatedAudioInput       STT + LLM + TTS (pre-warmed, audio gated when asleep)
       │  ros_tool_server        ROS 2 tools → /llm_cmd_vel, animations, camera
       ▼
  robot.service                 neural_controller, camera, cmd_vel_mux, …
       │
       ▼
  USB speaker (PipeWire default sink)
```

### Three systemd services / 三個 systemd 服務

| Service | Role | 說明 |
|---------|------|------|
| `pupster-wake.service` | Always-on wake word | 常駐聆聽「Hey Jarvis」，觸發 gate |
| `llm-agent.service` | `python src/agent.py start` | 預熱 agent；idle 時 audio muted |
| `robot.service` | ROS 2 bringup | 馬達、相機、mux、導航等 |

### Pre-warm gating / 預熱閘門機制

**English:** `llm-agent` stays running so STT/LLM/TTS stay warm (~1–2 s response after wake). While idle, `_GatedAudioInput` drops mic frames so OpenAI Realtime does not bill for silence. `pupster_wake` touches `/tmp/pupster_gate`; `start_gate_watcher()` in `pupster.py` toggles the gate without restarting the session.

**中文：** agent 常駐以保持 STT/LLM/TTS 預熱（喚醒後約 1–2 秒可回應）。閒置時 `_GatedAudioInput` 阻擋麥克風，避免 OpenAI Realtime 對靜音計費。喚醒服務寫入 `/tmp/pupster_gate`，gate watcher 切換 AWAKE/SLEEP，無需重啟 session。

---

## Pipeline / 語音管線

Current production design (`agent.py`: `AGENT_DESIGN = "openai-cartesia"` — name kept for history; TTS is **DashScope**, not Cartesia):

```
Mic → OpenAI Realtime (gpt-realtime, text modality, server VAD)
    → dashscope_tts.TTS (Qwen Realtime WebSocket)
    → Speaker
    ↔ function tools → RosToolServer → ROS topics / services
```

| Stage | Provider | Notes |
|-------|----------|-------|
| STT + LLM | OpenAI Realtime API | `modalities=["text"]`, server-side VAD |
| TTS | **DashScope Qwen Realtime** | Replaced Cartesia (402 quota exhausted) |
| Alt LLM | Gemini Live | `AGENT_DESIGN = "google-cartesia"` (optional) |
| Robot I/O | `ros_tool_server.py` | cmd_vel, animations, battery, camera tools |

### TTS migration summary / TTS 遷移摘要

| | Cartesia (old) | DashScope Qwen (current) |
|---|----------------|---------------------------|
| Status | Removed — HTTP 402 | Production default |
| Emotion | Inline `<emotion>` XML tags | Session `instructions` (no inline tags) |
| Voice clone | N/A | `qwen-tts-vc-*` via Singapore intl endpoint |
| Plugin | `livekit.plugins.cartesia` | `src/dashscope_tts.py` (LiveKit-compatible) |
| Tag leak fix | N/A | `_clean_text()` strips legacy Cartesia SSML |

Default Jarvis persona (`DEFAULT_INSTRUCTIONS` in `dashscope_tts.py`):

> 請模仿一隻機器狗說話：聲音略帶機械感、節奏穩定，但偶有一點點俏皮的上揚，整體保持冷靜可靠。

Voice clones (`qwen3-tts-vc-realtime`) respond weakly to “excited/sad” instructions but work well for calm/robotic tone — suitable for Jarvis.

---

## Vision tools / 視覺工具

Two ROS-backed tools in `PupsterAgent` / `RosToolServer`:

| Tool | Latency | Use case |
|------|---------|----------|
| `get_camera_image` | ~1–2 s | “What do you see?” — uses **gpt-realtime** multimodal on compressed camera frame |
| `analyze_camera_image` | ~3.5–4 s | Navigation / bounding boxes — fisheye unwarp + **Gemini 2.5 Flash** |

System prompt (`system_prompt.md`) defaults to `get_camera_image`; escalates to `analyze_camera_image` for navigation waypoints.

---

## ROS integration / ROS 整合

`ros_tool_server.py` bridges LiveKit function tools to ROS 2:

| Tool category | Examples |
|---------------|----------|
| Locomotion | Publish `/llm_cmd_vel` (via mux priority below teleop) |
| Animations | `/animation_controller_py/animation_select` |
| Status | Battery, controller activate/deactivate |
| Vision | Camera subscribe, fisheye → equirect, Gemini / Realtime |

**cmd_vel_mux priority** (see `neural_controller/launch/config.yaml`):

1. `/teleop_cmd_vel` (gamepad)
2. `/nav_cmd_vel` (Nav2)
3. `/llm_cmd_vel` (voice agent)
4. `/person_following_cmd_vel`

Nav2 remaps `cmd_vel` → `/nav_cmd_vel` in `pupper_nav/launch/nav.launch.py`.

---

## Key files / 主要檔案

| File | Purpose |
|------|---------|
| `src/agent.py` | Entry point, session design, gate watcher startup |
| `src/pupster.py` | `PupsterAgent`, tools, `_GatedAudioInput`, session factories |
| `src/dashscope_tts.py` | DashScope Qwen Realtime TTS LiveKit plugin |
| `src/ros_tool_server.py` | ROS 2 tool implementation |
| `src/system_prompt.md` | Jarvis personality, tool selection, emotion guidance |
| `src/gemini_interface.py` | Gemini vision API for `analyze_camera_image` |
| `.env.local` | API keys and TTS tuning (on Pi, not committed) |

Wake word deploy scripts live in **local-only** `scripts_local/pupster_wake/` (not in git). See workspace `PUPSTER_NOTES.md` for operator runbook.

---

## Environment / 環境變數

Copy `.env.example` → `.env.local` on the robot:

```bash
# Required
OPENAI_API_KEY=...
DASHSCOPE_API_KEY=...

# LiveKit (console/dev modes; Pi production uses local worker)
LIVEKIT_URL=...
LIVEKIT_API_KEY=...
LIVEKIT_API_SECRET=...

# TTS (DashScope Qwen Realtime)
OPENTALKING_TTS_VOICE=              # e.g. Cherry or qwen-tts-vc-twgirl-voice-...
OPENTALKING_QWEN_TTS_WS_URL=wss://dashscope-intl.aliyuncs.com/api-ws/v1/realtime
OPENTALKING_QWEN_TTS_INSTRUCTIONS=  # optional style override
OPENTALKING_QWEN_TTS_SPEECH_RATE=   # 0.5–2.0
OPENTALKING_QWEN_TTS_VOLUME=        # 0–100

# Optional vision
GOOGLE_API_KEY=                     # for analyze_camera_image (Gemini)
```

**Region note:** Singapore API keys must use `dashscope-intl.aliyuncs.com`; Beijing keys use `dashscope.aliyuncs.com`. Voice clones are locked to the enrollment region and model family (`qwen3-tts-vc-realtime-*`).

---

## Agent design switch / 切換 LLM 後端

Edit `AGENT_DESIGN` in `src/agent.py`:

| Value | LLM | TTS |
|-------|-----|-----|
| `openai-cartesia` | OpenAI Realtime (text) | DashScope Qwen (**default**) |
| `google-cartesia` | Gemini Live | DashScope Qwen |
| `openai-realtime` | OpenAI Realtime (audio) | OpenAI built-in audio |
| `cascade` | Disabled on Pi (VAD model issues) | — |

After changes on Pi:

```bash
sudo systemctl restart llm-agent
```

---

## Development setup / 開發環境

```console
cd agent-starter-python
uv sync
cp .env.example .env.local   # fill in keys
uv run python src/agent.py download-files   # Silero VAD etc. if needed
uv run python src/agent.py console          # terminal test
uv run python src/agent.py dev              # with web frontend
```

Production on Pi:

```bash
sudo systemctl start llm-agent
journalctl -u llm-agent -f
```

Expected startup log (TTS):

```
DashScope TTS: voice=... model=qwen3-tts-vc-realtime-2026-01-15 ...
[gate] initial: SLEEP
```

---

## Change log (high level) / 修改摘要

1. **Wake + pre-warm** — `pupster_wake` + `/tmp/pupster_gate` + `_GatedAudioInput` for fast response without idle API cost.
2. **TTS: Cartesia → DashScope Qwen Realtime** — custom `dashscope_tts.py`; strips legacy `<emotion>` tags from LLM output.
3. **Voice clone** — Taiwan voice via `clone_voice.py` (local script); intl endpoint + `qwen3-tts-vc-realtime-2026-01-15`.
4. **Persona tuning** — R03 robot-dog `instructions`; env overrides for rate/volume without code changes.
5. **Vision dual-path** — fast `get_camera_image` (Realtime) vs slow `analyze_camera_image` (Gemini + unwarp).
6. **ROS tools** — locomotion, animations, battery, person-following hooks via `ros_tool_server`.
7. **USB audio** — PipeWire default sink for USB speaker; mic via `PUPSTER_MIC=pulse` in wake service.

For detailed operator notes (homing, battery, quirks): see `PUPSTER_NOTES.md` in the workspace root (local reference doc).

---

## Upstream / License

Based on [LiveKit Agents Starter - Python](https://github.com/livekit/agents).  
MIT License — see [LICENSE](LICENSE).
