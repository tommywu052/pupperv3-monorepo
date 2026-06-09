# Pupper v3 — Pi `/home/pi` HTTP bridge (OpenClaw / remote control)

These files run **on the Raspberry Pi** at `/home/pi/` (not inside `ros2_ws`). They expose a FastAPI HTTP server so a **remote agent host** (OpenClaw, Jetson Thor unified control, Windows dev machine) can drive the robot without embedding ROS on the client.

這些檔案部署在 Pi 的 `/home/pi/`，提供 HTTP/JSON API，供 **OpenClaw 或遠端控制主機**（如 Jetson Thor）呼叫，無需在客戶端跑 ROS。

---

## Architecture / 架構

```
OpenClaw / Thor driver / curl
        │  HTTP :8095
        ▼
pupper-bridge.service  →  pupper_bridge.py  (FastAPI + rclpy)
        │                      ├─ /move, /stand, /animation  → ROS topics & controller_manager
        │                      ├─ /tts  → subprocess → pupper_tts_worker*.py → paplay
        │                      └─ /camera/describe  → capture + VLM (LLM_BASE_URL on Thor)
        ▼
robot.service (neural_controller, camera, cmd_vel_mux, …)
```

**Distinct from Pupster (`llm-agent`):** Pupster is the on-robot **wake-word + LiveKit** voice stack under `ai/llm-ui/agent-starter-python/`. The HTTP bridge is for **external orchestrators** that call REST endpoints (typical OpenClaw tool integration).

**與 Pupster 語音 agent 的差異：** Pupster 是 Pi 上常駐的喚醒詞 + LiveKit 對話；HTTP bridge 給 **OpenClaw 等外部編排器** 用 REST 控制機器人。

---

## Files / 檔案

| File | Role |
|------|------|
| `pupper_bridge.py` | Main FastAPI server (`PUPPER_BRIDGE_PORT`, default **8095**) |
| `pupper_tts_worker.py` | Subprocess TTS via **DashScope Qwen instruct-flash** → WAV → `paplay` |
| `pupper_tts_worker_cosyvoice.py` | Subprocess TTS via **CosyVoice HTTP** on Thor (`COSYVOICE_TTS_URL`) |
| `pupper-bridge.service.example` | systemd unit template (copy to `/etc/systemd/system/`) |

---

## HTTP endpoints

| Method | Path | Description |
|--------|------|-------------|
| GET | `/health` | Liveness |
| POST | `/move` | `{"vx","vy","wz","duration"}` → `/llm_cmd_vel` or teleop path |
| POST | `/stop` | Zero velocity |
| POST | `/stand` | Activate `neural_controller` |
| POST | `/sit` | Deactivate walking controllers |
| POST | `/animation` | `{"name":"sneeze"}` → `/animation_controller_py/animation_select` |
| GET | `/animations` | List animation nicknames |
| GET | `/camera/capture` | Base64 JPEG (fresh grab from `/camera/image_raw/compressed`) |
| POST | `/camera/describe` | Capture + **VLM** text description (designed for OpenClaw; returns text-only by default to save tokens) |
| POST | `/tts` | `{"text","backend":"dashscope\|cosyvoice"}` → speaker |
| POST | `/volume` | PipeWire volume |

`/camera/describe` accepts per-request overrides: `vlm_url`, `vlm_model`, `vlm_key` (so Thor can inject current LAN IP without redeploying the bridge).

---

## TTS backends

| Backend | Worker | When to use |
|---------|--------|-------------|
| `dashscope` (default) | `pupper_tts_worker.py` | Pi speaks directly via DashScope WebSocket (voice `Kai`, instruct-flash model) |
| `cosyvoice` | `pupper_tts_worker_cosyvoice.py` | Thor hosts CosyVoice HTTP server; Pi only plays returned WAV |

Set `TTS_BACKEND` in systemd or pass `"backend"` in POST `/tts` body.

---

## Deploy to Pi

```bash
scp pi_home/pupper_bridge.py pi_home/pupper_tts_worker*.py pi@<pi-ip>:/home/pi/
ssh pi@<pi-ip> "pip install fastapi uvicorn dashscope"

# systemd (edit API keys in the unit first)
sudo cp pupper-bridge.service.example /etc/systemd/system/pupper-bridge.service
sudo systemctl daemon-reload
sudo systemctl enable --now pupper-bridge.service
```

Requires `robot.service` running (bridge sources `pupperv3-monorepo/ros2_ws/install/setup.bash`).

---

## Environment variables (common)

| Variable | Purpose |
|----------|---------|
| `PUPPER_BRIDGE_HOST` | Bind address (default `0.0.0.0`) |
| `PUPPER_BRIDGE_PORT` | Default `8095` |
| `DASHSCOPE_API_KEY` | DashScope TTS |
| `OPENTALKING_TTS_VOICE` | e.g. `Kai` |
| `TTS_BACKEND` | `dashscope` or `cosyvoice` |
| `COSYVOICE_TTS_URL` | Thor CosyVoice endpoint |
| `LLM_BASE_URL` / `LLM_MODEL` / `LLM_API_KEY` | Default VLM for `/camera/describe` |

Never commit real API keys; set them only in `/etc/systemd/system/pupper-bridge.service` or Pi `.env`.
