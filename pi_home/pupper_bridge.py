"""Pupper v3 HTTP Bridge — runs ON the Pupper Raspberry Pi.

Exposes ROS2 robot commands as a simple HTTP/JSON API so the unified
Robot Control server (running on Jetson Thor or Windows host) can
control the Pupper over the network.

Deploy to Pupper:
    scp pupper_bridge.py pi@<pupper-ip>:/home/pi/pupper_bridge.py
    ssh pi@<pupper-ip> "pip install fastapi uvicorn dashscope && python pupper_bridge.py"

Or install as systemd service (see deploy script).

Endpoints:
    GET  /health
    POST /move          {"vx":0.5, "vy":0, "wz":0, "duration":2.0}
    POST /stop
    POST /stand         (activate walking controller)
    POST /sit           (deactivate all controllers -> robot sits)
    POST /animation     {"name":"sneeze"}
    GET  /animations
    GET  /camera/capture (returns base64 JPEG)
    POST /tts           {"text":"hello"} (DashScope Qwen TTS -> speaker)
    POST /volume        {"volume":0.5}
    POST /camera/describe  {"prompt":"..."} (VLM for OpenClaw / remote agents)
"""

import asyncio
import base64
import json
import logging
import os
import subprocess
import tempfile
import time
import wave
from contextlib import asynccontextmanager
from typing import Optional, Any

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from controller_manager_msgs.srv import SwitchController, ListControllers

from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse
import uvicorn
import threading

BRIDGE_HOST = os.environ.get("PUPPER_BRIDGE_HOST", "0.0.0.0")
BRIDGE_PORT = int(os.environ.get("PUPPER_BRIDGE_PORT", "8095"))

# DashScope Qwen TTS config
DASHSCOPE_API_KEY = os.environ.get("DASHSCOPE_API_KEY", "")
DASHSCOPE_WS_URL = os.environ.get(
    "OPENTALKING_QWEN_TTS_WS_URL",
    "wss://dashscope-intl.aliyuncs.com/api-ws/v1/realtime",
)
TTS_VOICE = os.environ.get("OPENTALKING_TTS_VOICE", "Kai")
# Built-in male voice + instruct-flash model (supports built-in voices +
# `instructions`). vc-realtime would only accept custom-cloned voice IDs.
TTS_MODEL = "qwen3-tts-instruct-flash-realtime"

WALKING_CONTROLLER = "neural_controller"
AVAILABLE_CONTROLLERS = {
    "neural_controller",
    "neural_controller_three_legged",
    "forward_kp_controller",
    "forward_position_controller",
    "forward_kd_controller",
}
ANIMATION_CONTROLLER = "animation_controller_py"
ANIMATION_TOPIC = f"/{ANIMATION_CONTROLLER}/animation_select"

ANIMATIONS = {
    "twerk": "twerk_recording_2025-09-04_16-14-51_0",
    "lie_sit_lie": "lie_sit_lie_recording_2025-09-03_12-44-08_0",
    "stand_sit_shake_sit_stand": "stand_sit_shake_sit_stand_recording_2025-09-03_12-47-18_0",
    "upward_dog": "upward_dog_recording_2025-10-22_17-17-07",
    "superman": "superman_recording_2025-10-22_17-47-41",
    "pee": "pee2_recording_2025-10-22_17-41-45",
    "lie_downward_dog": "lie_downward_dog_recording_2025-09-04_16-08-00_0",
    "sneeze": "sneeze_recording_2025-09-04_16-13-54_0",
    "spider": "spider_recording_2025-09-04_16-12-38_0",
    "swim": "swim_recording_2025-09-04_16-10-45_0",
    "sajiao": "sajiao_recording_2026-05-19_23-35-53",
}

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("pupper_bridge")

# ROS2 node (initialized in lifespan)
_node: Optional[Node] = None
_executor: Optional[SingleThreadedExecutor] = None
_twist_pub = None
_animation_pub = None
_switch_ctrl_client = None
_list_ctrl_client = None
_latest_image: Optional[bytes] = None
_latest_image_time: float = 0.0  # monotonic time of last camera frame
_image_callback_count: int = 0
_ros_thread: Optional[threading.Thread] = None


def _ros_spin():
    """Run ROS2 executor in a background thread."""
    global _executor
    while rclpy.ok():
        _executor.spin_once(timeout_sec=0.1)


def _image_callback(msg):
    global _latest_image, _latest_image_time, _image_callback_count
    _latest_image = bytes(msg.data)
    _latest_image_time = time.monotonic()
    _image_callback_count += 1


@asynccontextmanager
async def lifespan(app: FastAPI):
    global _node, _executor, _twist_pub, _animation_pub
    global _switch_ctrl_client, _list_ctrl_client, _ros_thread

    rclpy.init()
    _node = Node("pupper_bridge")
    _executor = SingleThreadedExecutor()
    _executor.add_node(_node)

    _twist_pub = _node.create_publisher(Twist, "/cmd_vel", 10)
    _animation_pub = _node.create_publisher(String, ANIMATION_TOPIC, 10)
    _switch_ctrl_client = _node.create_client(
        SwitchController, "/controller_manager/switch_controller"
    )
    _list_ctrl_client = _node.create_client(
        ListControllers, "/controller_manager/list_controllers"
    )

    # IMPORTANT: pupper's camera_ros node publishes with RELIABLE QoS
    # (verified via `ros2 topic info -v`). We initially had
    # BEST_EFFORT here on the assumption that all camera publishers
    # use SENSOR_DATA, and that mismatch (RELIABLE pub + BEST_EFFORT
    # sub is technically allowed by DDS but flaky in practice) is what
    # produced the long-idle freeze symptom: subscription would work
    # for ~5-30 s after a fresh start, then stop delivering messages
    # even though `ros2 topic hz` still measured ~5 Hz. Match RELIABLE
    # so the DDS handshake is symmetric and stable.
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    image_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    _node.create_subscription(
        CompressedImage, "/camera/image_raw/compressed",
        _image_callback, image_qos,
    )

    _ros_thread = threading.Thread(target=_ros_spin, daemon=True)
    _ros_thread.start()

    logger.info(f"Pupper Bridge running on http://{BRIDGE_HOST}:{BRIDGE_PORT}")
    yield

    _node.destroy_node()
    rclpy.shutdown()


app = FastAPI(lifespan=lifespan)


def _ok(data=None, **extra):
    out = {"ok": True}
    if isinstance(data, dict):
        out.update(data)
    out.update(extra)
    return JSONResponse(out)


def _err(msg, code=500):
    return JSONResponse({"ok": False, "error": str(msg)}, status_code=code)


@app.get("/health")
async def health():
    return _ok({
        "service": "pupper_bridge",
        "ros_ok": rclpy.ok(),
        "has_image": _latest_image is not None,
    })


@app.post("/move")
async def move(req: Request):
    """Move with velocity. Body: {"vx":0.5, "vy":0, "wz":0, "duration":2.0}
    If duration is set, moves for that time then stops.
    If no duration, just sets velocity (caller must /stop later)."""
    body = await req.json()
    vx = float(body.get("vx", 0))
    vy = float(body.get("vy", 0))
    wz = float(body.get("wz", 0))
    duration = body.get("duration")

    twist = Twist()
    twist.linear.x = vx
    twist.linear.y = vy
    twist.angular.z = wz / 57.2958  # deg/s -> rad/s

    _twist_pub.publish(twist)

    if duration:
        duration = float(duration)
        await asyncio.sleep(duration)
        stop_twist = Twist()
        _twist_pub.publish(stop_twist)
        return _ok({"vx": vx, "vy": vy, "wz": wz, "duration": duration, "stopped": True})

    return _ok({"vx": vx, "vy": vy, "wz": wz})


@app.post("/stop")
async def stop():
    twist = Twist()
    _twist_pub.publish(twist)
    return _ok({"stopped": True})


@app.post("/stand")
async def stand():
    """Activate the walking controller (robot stands up)."""
    req = SwitchController.Request()
    req.activate_controllers = [WALKING_CONTROLLER]
    req.deactivate_controllers = list(AVAILABLE_CONTROLLERS - {WALKING_CONTROLLER})
    req.strictness = 1

    future = _switch_ctrl_client.call_async(req)
    rclpy.spin_until_future_complete(_node, future, timeout_sec=3.0)

    if future.done() and future.result() and future.result().ok:
        return _ok({"controller": WALKING_CONTROLLER, "state": "active"})
    return _err("Failed to activate walking controller")


@app.post("/sit")
async def sit():
    """Deactivate all controllers (robot sits/relaxes)."""
    req = SwitchController.Request()
    req.activate_controllers = []
    req.deactivate_controllers = list(AVAILABLE_CONTROLLERS)
    req.strictness = 1

    future = _switch_ctrl_client.call_async(req)
    rclpy.spin_until_future_complete(_node, future, timeout_sec=3.0)

    if future.done() and future.result() and future.result().ok:
        return _ok({"state": "deactivated"})
    return _err("Failed to deactivate controllers")


@app.post("/animation")
async def play_animation(req: Request):
    """Play a named animation. Body: {"name":"sneeze"}"""
    body = await req.json()
    name = body.get("name", "")
    if name not in ANIMATIONS:
        return _err(f"Unknown animation: {name}. Available: {list(ANIMATIONS.keys())}", 400)

    csv_name = ANIMATIONS[name]
    msg = String()
    msg.data = csv_name
    _animation_pub.publish(msg)
    return _ok({"animation": name, "csv": csv_name})


@app.get("/animations")
async def list_animations():
    return JSONResponse({"animations": list(ANIMATIONS.keys())})


def _grab_fresh_frame(timeout_s: float = 2.5) -> Optional[bytes]:
    """Spawn a one-shot Node+Executor pair to grab a fresh CompressedImage.

    Bypasses the long-lived `_image_callback` subscription that goes
    stale after a period of idle. Creates its OWN node, executor, and
    subscription, spins until a single message arrives, then tears
    everything down. This mirrors what `ros2 topic echo --once` does
    (which we verified works even when our long-lived subscription is
    silent), and avoids any state contention with the main `_node`
    that's already being spun by `_ros_thread`.
    """
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from rclpy.executors import SingleThreadedExecutor
    from threading import Event

    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    got = Event()
    grabbed: dict = {}

    def _cb(msg):
        if not got.is_set():
            grabbed["data"] = bytes(msg.data)
            got.set()

    fresh_node = Node("pupper_bridge_oneshot_cam")
    sub = fresh_node.create_subscription(
        CompressedImage, "/camera/image_raw/compressed", _cb, qos
    )
    fresh_exec = SingleThreadedExecutor()
    fresh_exec.add_node(fresh_node)
    deadline = time.monotonic() + timeout_s
    try:
        while not got.is_set() and time.monotonic() < deadline:
            fresh_exec.spin_once(timeout_sec=0.05)
        return grabbed.get("data")
    finally:
        try:
            fresh_node.destroy_subscription(sub)
            fresh_exec.remove_node(fresh_node)
            fresh_node.destroy_node()
        except Exception:  # noqa: BLE001
            pass


@app.get("/camera/capture")
async def camera_capture():
    """Return latest camera frame as base64 JPEG.

    Always uses `_grab_fresh_frame` because the long-lived subscription
    on `_node` repeatably stalls after idle (we verified the publisher
    is still pushing at 5 Hz via `ros2 topic hz` while our callback
    count was stuck). One-shot Node+Executor per call is bullet-proof.
    Falls back to the cached `_latest_image` if the fresh grab fails.
    """
    global _latest_image, _latest_image_time
    loop = asyncio.get_event_loop()
    fresh = await loop.run_in_executor(None, _grab_fresh_frame, 2.5)
    used_fresh = False
    if fresh is not None:
        _latest_image = fresh
        _latest_image_time = time.monotonic()
        used_fresh = True
    if _latest_image is None:
        return _err("No camera image available yet", 503)

    b64 = base64.b64encode(_latest_image).decode("ascii")
    return _ok({
        "format": "jpeg",
        "size": len(_latest_image),
        "b64": b64,
        "age_s": round(time.monotonic() - _latest_image_time, 2)
              if _latest_image_time else None,
        "frame_count": _image_callback_count,
        "fresh": used_fresh,
    })


# TTS backend dispatch. Resolved per request: body.backend -> env -> default.
# Adding a new backend means a new {name: worker_path} entry and a new worker
# script that takes (text [, spk_id]) and writes "OK <pcm_bytes>" to stdout.
_TTS_WORKERS = {
    "dashscope": "/home/pi/pupper_tts_worker.py",
    "cosyvoice": "/home/pi/pupper_tts_worker_cosyvoice.py",
}
_DEFAULT_TTS_BACKEND = os.environ.get("TTS_BACKEND", "dashscope").lower()


@app.post("/tts")
async def tts(req: Request):
    """Synthesize text and play.

    Backend selection order (highest priority first):
      1. POST body ``backend`` ("dashscope" or "cosyvoice")
      2. env ``TTS_BACKEND`` (set at service startup)
      3. default "dashscope"  -- preserves legacy behaviour

    Optional body fields:
      - ``spk_id`` (cosyvoice only, default "default")
    """
    body = await req.json()
    text = body.get("text", "")
    if not text:
        return _err("missing 'text'", 400)

    backend = (body.get("backend") or _DEFAULT_TTS_BACKEND).lower()
    worker_path = _TTS_WORKERS.get(backend)
    if worker_path is None:
        return _err(f"Unknown TTS backend: {backend}. Known: {list(_TTS_WORKERS)}", 400)
    if not os.path.exists(worker_path):
        return _err(f"TTS worker not found: {worker_path}", 500)

    argv = ["python3", worker_path, text]
    # Per-request env overrides (driver may inject these so SSID / IP changes
    # on Thor don't require a robot-side redeploy).
    extra_env = {}
    if backend == "cosyvoice":
        spk_id = body.get("spk_id") or os.environ.get("COSYVOICE_SPK_ID", "default")
        argv.append(spk_id)
        cv_url = body.get("cosyvoice_url")
        if cv_url:
            extra_env["COSYVOICE_TTS_URL"] = cv_url

    try:
        loop = asyncio.get_event_loop()
        proc = await loop.run_in_executor(
            None,
            lambda: subprocess.run(
                argv,
                capture_output=True, timeout=25,
                env={**os.environ, "DASHSCOPE_API_KEY": DASHSCOPE_API_KEY, **extra_env},
            ),
        )
        stdout = proc.stdout.decode(errors="replace").strip()
        stderr = proc.stderr.decode(errors="replace").strip()

        if proc.returncode != 0:
            logger.error(f"TTS worker failed ({backend}): {stderr}")
            return _err(f"TTS failed ({backend}): {stderr}")

        pcm_bytes = 0
        if stdout.startswith("OK"):
            parts = stdout.split()
            if len(parts) >= 2:
                pcm_bytes = int(parts[1])

        duration = pcm_bytes / 2 / 24000 if pcm_bytes else 0
        data = {
            "text": text,
            "duration": round(duration, 2),
            "pcm_bytes": pcm_bytes,
            "backend": backend,
        }
        if backend == "cosyvoice":
            data["spk_id"] = argv[3]
        return _ok(data)
    except subprocess.TimeoutExpired:
        return _err(f"TTS synthesis timeout (25s, backend={backend})")
    except Exception as e:
        logger.error(f"TTS failed ({backend}): {e}")
        return _err(f"TTS failed ({backend}): {e}")


@app.post("/volume")
async def set_volume(req: Request):
    """Set system volume (0.0-1.0) via wpctl."""
    body = await req.json()
    vol = float(body.get("volume", 0.5))
    try:
        subprocess.run(
            ["wpctl", "set-volume", "@DEFAULT_AUDIO_SINK@", str(vol)],
            check=True, capture_output=True,
        )
        return _ok({"volume": vol})
    except Exception as e:
        return _err(f"Volume set failed: {e}")


@app.post("/camera/describe")
async def camera_describe(req: Request):
    """Capture image + call VLM to describe the scene.

    Response is intentionally text-only (`{"description": "..."}`) — earlier
    versions also returned `image_base64` and `image_size`, but when an LLM
    agent (OpenClaw / Qwen) called this endpoint directly the ~120 KB b64
    image landed in chat context and blew through the 65 K token window
    after a few iterations. Callers that need the actual bytes should hit
    `/camera/capture` separately (it's the right endpoint for that). To
    re-enable the b64 in the response for debugging, send
    `{"include_image": true}` in the body.
    """
    global _latest_image, _latest_image_time
    body = await req.json()
    prompt = body.get("prompt", "請用繁體中文簡短描述你看到的畫面，2-3句話。")
    include_image = bool(body.get("include_image", False))

    # Match /camera/capture: use fresh-grab so describe sees the
    # current scene, not whatever was last cached.
    loop = asyncio.get_event_loop()
    fresh = await loop.run_in_executor(None, _grab_fresh_frame, 2.5)
    if fresh is not None:
        _latest_image = fresh
        _latest_image_time = time.monotonic()
    if _latest_image is None:
        return _err("No camera image available yet", 503)

    # VLM connection settings: per-request body override > env fallback.
    # The body override path mirrors the CosyVoice TTS pattern (driver
    # injects `cosyvoice_url`): driver runs on Thor and resolves Thor's
    # current LAN IP at request time, so the bridge does not need a
    # systemd redeploy when Thor moves SSID. Env values stay as the
    # last-resort fallback (used only if driver was bypassed, e.g.
    # somebody curl'd this endpoint directly from the bridge host).
    vlm_url = body.get("vlm_url") or os.environ.get("LLM_BASE_URL", "https://api.x.ai/v1")
    vlm_model = body.get("vlm_model") or os.environ.get("LLM_MODEL", "grok-4.20-0309-non-reasoning")
    vlm_key = body.get("vlm_key") or os.environ.get("LLM_API_KEY") or os.environ.get("GROK_KEY", "")

    if not vlm_key:
        return _err("No VLM API key configured (LLM_API_KEY or GROK_KEY)", 500)

    # Pupper camera is a fisheye lens that outputs grayscale-as-RGB at
    # 1400x1050 (~220 KB JPEG). Empirically Qwen3.6-NVFP4's vision
    # encoder either refuses the frame ("畫面無法辨識") or hallucinates
    # detail-rich but invented scenes (e.g. "4 white plastic bottles
    # with red labels" when there are 0). Pre-process: center-crop the
    # circular fisheye view, then resize to 768px on the long edge so
    # the VLM gets an image closer to its training distribution.
    b64_img = _preprocess_for_vlm(_latest_image)

    try:
        loop = asyncio.get_event_loop()
        description = await loop.run_in_executor(
            None, _call_vlm, b64_img, prompt, vlm_key, vlm_model, vlm_url
        )
        payload = {"description": description, "image_size": len(_latest_image)}
        if include_image:
            payload["image_base64"] = b64_img
        return _ok(payload)
    except Exception as e:
        logger.error(f"VLM failed: {e}")
        return _err(f"VLM failed: {e}")


def _preprocess_for_vlm(jpeg_bytes: bytes) -> str:
    """Crop fisheye + resize to a VLM-friendly size; return base64 JPEG.

    Falls back to the raw bytes (re-encoded as base64) if PIL isn't
    available or any step fails — we never want preprocessing to be a
    new failure mode.
    """
    try:
        from PIL import Image, ImageOps
        import io
        im = Image.open(io.BytesIO(jpeg_bytes))
        w, h = im.size
        # Pupper's camera is physically mounted upside-down on the head,
        # so every frame is rotated 180°. Sending a flipped scene to the
        # VLM is a major source of hallucination (we saw "robot dog
        # standing in the center" — the model was looking at a chair
        # whose silhouette only resembles a quadruped when inverted).
        # Rotate it right-side-up first.
        im = im.rotate(180)
        # Center-crop a square just inside the circular fisheye so we
        # drop the black corners that Qwen sometimes treats as "scene
        # is empty / dark". The circular FOV diameter ~= h on pupper's
        # 1400x1050 frame, so a square h×h around the center captures
        # essentially all the useful pixels.
        s = min(w, h)
        left = (w - s) // 2
        top = (h - s) // 2
        im = im.crop((left, top, left + s, top + s))
        # Long-edge target 768 — Qwen3-VL family tile size is 28×28 with
        # max ~768 along an edge; matches its native processing better
        # than the raw 1050.
        target = 768
        if im.size[0] != target:
            im = im.resize((target, target), Image.LANCZOS)
        # Pupper output is grayscale-as-RGB with mean ~75/255 — pretty
        # dark. autocontrast (cutoff=2) brings objects out of the murk;
        # we then nudge it toward higher mid-tones with equalize so the
        # VLM has more distinctive features to latch onto.
        im = ImageOps.autocontrast(im, cutoff=2)
        buf = io.BytesIO()
        im.convert("RGB").save(buf, format="JPEG", quality=88)
        return base64.b64encode(buf.getvalue()).decode("ascii")
    except Exception as e:  # noqa: BLE001
        logger.warning(f"VLM preprocess failed, sending raw: {e}")
        return base64.b64encode(jpeg_bytes).decode("ascii")


def _call_vlm(b64_jpeg: str, prompt: str, api_key: str, model: str, api_url: str) -> str:
    """Call OpenAI-compatible Vision API (Grok / vLLM-Qwen3.6 / Nemotron Nano-Omni).

    Qwen3.6 quirk: when `enable_thinking` is on (default), the textual answer
    lands in `message.reasoning`, and `message.content` is `null`. We pass
    `chat_template_kwargs.enable_thinking=false` via `extra_body` to force the
    answer into `content`. As a safety net we also fall back to `reasoning`
    if `content` ends up empty.
    """
    import urllib.request
    import urllib.error

    url = f"{api_url}/chat/completions"
    # Bridge-level system prompt is intentionally MINIMAL.
    #
    # Earlier iterations tried to be helpful by:
    #   - telling the model "you are a robot dog vision system"
    #     (made it open every reply with "您好！我是机器狗的视觉系统")
    #   - telling it "if image is too dark, reply 畫面無法辨識"
    #     (gave it a too-easy escape; it returned 無法辨識 even when
    #      a clear water bottle was visible directly in front of pupper)
    #
    # Diagnostic verified: same image, same vLLM endpoint, same model.
    # WITH the long bridge system prompt + neutral prompt: "畫面無法辨識".
    # WITHOUT (direct vLLM call): "畫面中地板上確實有一個瓶子...".
    # → bridge system prompt was actively SUPPRESSING good answers.
    #
    # Now: language constraint only. The caller's prompt (e.g. the
    # JSON template from visual_search.py's _build_search_prompt) is
    # already detailed enough to drive the answer.
    payload = json.dumps({
        "model": model,
        "messages": [
            {
                "role": "system",
                "content": "請使用繁體中文回答。直接描述使用者圖片中看到的內容，不要自我介紹。",
            },
            {
                "role": "user",
                "content": [
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64_jpeg}"}},
                    {"type": "text", "text": prompt},
                ],
            }
        ],
        "max_tokens": 300,
        "temperature": 0.2,
        "chat_template_kwargs": {"enable_thinking": False},
    }).encode()

    req = urllib.request.Request(
        url, data=payload, method="POST",
        headers={
            "Content-Type": "application/json",
            "Authorization": f"Bearer {api_key}",
        },
    )
    resp = urllib.request.urlopen(req, timeout=30)
    data = json.loads(resp.read())
    msg = data["choices"][0]["message"]
    text = msg.get("content")
    if not text:
        text = msg.get("reasoning") or ""
    return text


if __name__ == "__main__":
    uvicorn.run(app, host=BRIDGE_HOST, port=BRIDGE_PORT, log_level="info")
