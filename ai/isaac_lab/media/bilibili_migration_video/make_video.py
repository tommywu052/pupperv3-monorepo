"""Build the ~5 minute Bilibili video for the Pupper MJX -> Isaac Lab talk.

Pipeline:
  1. Render chapter / code / config visuals as 1080p VS Code-style frames.
  2. Generate Chinese narration. Default backend is DashScope Qwen TTS using the
     cloned **Tommy** voice (``--tts qwen``); ``--tts edge`` falls back to EdgeTTS.
  3. Build one MP4 segment per scene with ffmpeg. Static scenes loop a frame
     under the narration; the two demo scenes loop the real training / sim2real
     clip under the narration.
  4. Concatenate, write .srt and burn a subtitled copy.

Demo clips (auto-detected, override with --clip scene_id=path.mp4):
  09_training_video : ai/isaac_lab/isaac_rl_result_pupper.mp4
  14_sim2real_video : ai/isaac_lab/pupper_sim2real_deploy.mp4
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import re
import shutil
import subprocess
import sys
import wave
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parent
OUT = ROOT / "out"
ISAAC_LAB = ROOT.parents[1]  # ai/isaac_lab

# DashScope Qwen TTS (cloned Tommy voice). Mirrors opentalking offline_tts.py.
OPENTALKING = Path(r"C:\Nvidia\opentalking")
TOMMY_VOICE = "qwen-tts-vc-tommyvoice-voice-20260507113701487-f2b2"
TOMMY_MODEL = "qwen3-tts-vc-realtime-2026-01-15"
TTS_SR = 16000

# EdgeTTS fallback.
EDGE_VOICE = "zh-TW-HsiaoChenNeural"
EDGE_RATE = "-10%"

W, H = 1920, 1080
FPS = 30


@dataclass(frozen=True)
class Scene:
    scene_id: str
    title: str
    narration: str
    bullets: tuple[str, ...]
    code_title: str = ""
    code: str = ""
    accent: tuple[int, int, int] = (0, 190, 180)
    default_clip: str = ""
    cover: bool = False
    cover_lines: tuple[str, ...] = ()
    screenshot: str = ""  # absolute path to a VS Code screenshot to embed instead of code panel
    extra_image: str = ""  # optional second image placed small in the lower-left empty area
    link: str = ""  # optional URL shown (clickable) as a footer on the slide


SCENES: list[Scene] = [
    Scene(
        "02_agenda",
        "Agenda",
        "大家好，這支影片用五分鐘，帶你把 Pupper 從 MuJoCo MJX 遷移到 Isaac Lab，並完成 sim2real。"
        "內容分成五段：MJX 參數與 Isaac Lab 設定、遷移架構、Isaac Lab 訓練要點、"
        "policy 部署真機要點，最後做總結。",
        (
            "1. MJX 參數與 Isaac Lab 設定",
            "2. 遷移架構",
            "3. Isaac Lab 訓練要點",
            "4. Policy 部署真機要點",
            "5. 總結",
        ),
        "ai/isaac_lab/README.md",
        "",
        (255, 181, 71),
        screenshot=str(ISAAC_LAB / "isaac-lab-folder-overview.png"),
    ),
    Scene(
        "03_architecture",
        "遷移架構",
        "先看架構。訓練從 Brax PPO 換成 rsl_rl PPO，物理從 MJX 換成 PhysX，模型由 URDF 轉成 USD。"
        "網路採不對稱 actor-critic，actor 只看真機可量測的觀測，最後匯出 ONNX 到 Raspberry Pi 部署。",
        (
            "Brax PPO -> rsl_rl PPO",
            "MJX -> PhysX GPU sim",
            "URDF -> USD (convert script)",
            "Asymmetric actor-critic -> ONNX -> ROS 2",
        ),
        "scripts/convert_pupper_urdf.py",
        """# MJX / Brax            Isaac Lab / PhysX
#   train.py        ->    scripts/train.py (rsl_rl)
#   MJCF            ->    URDF -> USD
#   actor obs       ==    deployable 45-dim contract
#   critic obs      +=    privileged base lin vel
# export: actor-only policy.onnx -> Pi (ROS 2)""",
        (118, 185, 0),
        screenshot=str(ISAAC_LAB / "isaac_lab_architecture_tree.png"),
        extra_image=str(ISAAC_LAB / "pupper_usd_isaacsim.png"),
    ),
    Scene(
        "04_contract",
        "固定 Policy Contract",
        "第一步是固定 policy 契約。Actor 觀測四十五維，只用真機可量測的量："
        "IMU 角速度、投影重力、速度命令、關節角和速度，加上一個 action。"
        "動作十二維，乘零點二五加到站姿，就是位置目標。",
        (
            "Actor obs: 45 dims (real-measurable)",
            "Action: raw 12 dims",
            "target = default + 0.25 * action",
        ),
        "SIM2REAL.md  (#1 contract)",
        """obs(45) = base_ang_vel(3)
        + projected_gravity(3)
        + velocity_commands(3)
        + (joint_pos - default)(12)
        + joint_vel(12)
        + last_action(12)

target = default_joint_pos + 0.25 * action
control = 50 Hz, physics = 200 Hz""",
        (139, 223, 120),
        screenshot=str(ISAAC_LAB / "PupperObservationCfg.png"),
    ),
    Scene(
        "05_physx_drive",
        "PhysX Drive 對齊 MJX PD",
        "在 Isaac Lab，policy 輸出位置目標，由 PhysX 的 ImplicitActuator 產生 torque。"
        "stiffness 是 KP 五，damping 是 KD 零點二五，physics 兩百赫茲、控制五十赫茲，對齊 MuJoCo。",
        (
            "ImplicitActuator: high-rate PhysX PD",
            "KP = 5.0, KD = 0.25",
            "effort_limit = 3.0 N*m, vel_limit = 30",
        ),
        "pupper_isaaclab/assets/pupper.py",
        """actuators={
    "legs": ImplicitActuatorCfg(
        joint_names_expr=[".*"],
        effort_limit_sim=3.0,
        velocity_limit_sim=30.0,
        stiffness={".*": KP},   # KP = 5.0
        damping={".*": KD},     # KD = 0.25
    ),
}""",
        (255, 121, 121),
    ),
    Scene(
        "06_env_cfg",
        "Isaac Lab Env 設定",
        "把 MJX 的命令範圍和時間尺度搬過來：physics 五毫秒、decimation 四，控制五十赫茲，episode 十秒。"
        "速度命令涵蓋前後零點七五、側移零點五、yaw 正負二。",
        (
            "sim.dt = 0.005, decimation = 4 (50 Hz)",
            "episode_length_s = 10.0",
            "vx +/-0.75, vy +/-0.5, wz +/-2.0",
        ),
        "tasks/locomotion/pupper_env_cfg.py",
        """self.sim.dt = 0.005
self.decimation = 4
self.episode_length_s = 10.0

cmd.ranges.lin_vel_x = (-0.75, 0.75)
cmd.ranges.lin_vel_y = (-0.5, 0.5)
cmd.ranges.ang_vel_z = (-2.0, 2.0)""",
        (153, 128, 255),
        screenshot=str(ISAAC_LAB / "PupperEnvConfig.png"),
    ),
    Scene(
        "07_training_recipe",
        "Sim2Real Recipe",
        "讓實機真正走起來的關鍵，是補回 MJX 的 sim2real 配方：一步 action 延遲，加上致動器增益隨機化。"
        "只有延遲容易翻，只有隨機化會凍住，兩個一起才會穩。",
        (
            "1-step action latency: delay_prob = 0.8",
            "stiffness scale: 0.6 - 1.1",
            "damping scale: 0.8 - 1.5",
        ),
        "tasks/locomotion/delayed_action.py",
        """self.actions.joint_pos = DelayedJointPositionActionCfg(
    scale=0.25,
    use_default_offset=True,
    delay_prob=0.8,
)

randomize_actuator_gains(
    stiffness_distribution_params=(0.6, 1.1),
    damping_distribution_params=(0.8, 1.5),
)""",
        (0, 190, 180),
    ),
    Scene(
        "08_ppo",
        "PPO 訓練要點",
        "訓練用 rsl_rl PPO，採不對稱 actor-critic。"
        "平地約六百個 iteration 就能得到可部署版本，崎嶇地形拉到一千五百。",
        (
            "Asymmetric actor-critic",
            "Flat: 600 iters, [256, 128, 128]",
            "Rough: 1500 iters, [512, 256, 128]",
        ),
        "agents/rsl_rl_ppo_cfg.py",
        """policy = RslRlPpoActorCriticCfg(
    actor_obs_normalization=True,
    critic_obs_normalization=True,
    actor_hidden_dims=[256, 128, 128],
    critic_hidden_dims=[256, 128, 128],
)
# PupperFlat: max_iterations = 600
# PupperRough: max_iterations = 1500""",
        (255, 181, 71),
        screenshot=str(ISAAC_LAB / "rsl_rl_ppo_pupper.png"),
    ),
    Scene(
        "08b_training_progress",
        "訓練過程縮時",
        "先看訓練過程。這段縮時可以看到 policy 從一開始亂動、站不穩，"
        "逐步學會協調四條腿，最後收斂成穩定步態。",
        (
            "Time-lapse over training",
            "亂動 -> 協調 -> 穩定步態",
            "rsl_rl PPO on flat terrain",
        ),
        "videos/pupper_training_progression.mp4",
        "",
        (139, 223, 120),
        default_clip=str(ISAAC_LAB / "videos" / "pupper_training_progression.mp4"),
    ),
    Scene(
        "09_training_video",
        "Isaac Lab 訓練成果",
        "這是 Isaac Lab 的訓練成果。Policy 穩定追蹤速度命令，步態收斂成乾淨的 limit cycle。",
        (
            "Isaac Lab / PhysX playback",
            "Policy tracks velocity command",
            "Stable gait limit cycle",
        ),
        "isaac_rl_result_pupper.mp4",
        "",
        (139, 223, 120),
        default_clip=str(ISAAC_LAB / "isaac_rl_result_pupper.mp4"),
    ),
    Scene(
        "10_export",
        "匯出 ONNX",
        "訓練完用 play 腳本匯出 ONNX。關鍵是 normalizer 會包進 ONNX，部署端餵原始觀測就好；"
        "也因此，觀測每一維順序都必須正確。",
        (
            "play.py exports actor only",
            "Normalizer embedded in ONNX",
            "Deploy feeds raw observation",
        ),
        "scripts/play.py  (export)",
        """export_policy_as_onnx(
    policy_nn,
    normalizer=obs_normalizer,
    path=export_dir,
    filename="policy.onnx",
)
# deploy side: raw obs -> ONNX -> action""",
        (0, 194, 255),
    ),
    Scene(
        "11_deploy",
        "Raspberry Pi 部署",
        "真機在 Raspberry Pi 上跑 onnxruntime、用 ROS 2。"
        "節點訂閱 joint states、IMU 和 cmd vel，推論後發布到 position、kp、kd 三個 controller，由馬達 PD 產生 torque。",
        (
            "onnxruntime CPU @ 50 Hz",
            "sub: /joint_states, /imu, /cmd_vel",
            "pub: forward_position / kp / kd",
        ),
        "deploy/pupper_onnx_node.py",
        """pub /forward_position_controller/commands
pub /forward_kp_controller/commands
pub /forward_kd_controller/commands

target_pol = default_pol + 0.25 * action
target_hw  = target_pol[INV_POL2HW]""",
        (0, 194, 255),
    ),
    Scene(
        "12_joint_order",
        "最大坑：Joint Order",
        "最大的坑是關節順序。MJX 依腿分組，但 Isaac Lab 經 PhysX 依層級重排。"
        "Sim 裡自洽所以正常，一到真機，action 就打到錯的腿。"
        "修法是在部署端加入 permutation，免重訓、免重匯出。",
        (
            "MJX / hardware: grouped by leg",
            "Isaac Lab / PhysX: grouped by joint level",
            "Fix: q_hw -> q_pol, target_pol -> target_hw",
        ),
        "deploy/pupper_onnx_node.py  (PERM)",
        """PERM_HW2POL = [9,6,3,0,10,7,4,1,11,8,5,2]
INV_POL2HW  = [3,7,11,2,6,10,1,5,9,0,4,8]

q_pol  = q_hw[PERM_HW2POL]
obs    = [..., q_pol - default_pol, qd_pol, last_action]
target_hw = target_pol[INV_POL2HW]""",
        (255, 121, 121),
    ),
    Scene(
        "13_safety",
        "真機安全與操控",
        "部署先 dry run，只讀感測、不送馬達。正式啟動從目前姿態 ramp 到站姿再淡入 policy；"
        "偵測翻倒就觸發 e-stop。鍵盤用 WSAD，搖桿用方塊鍵啟動。",
        (
            "Dry-run first: no motor command",
            "init ramp -> fade-in -> policy",
            "Tip-over e-stop + soft limp",
        ),
        "deploy/README.md",
        """python3 pupper_onnx_node.py --duration 12
python3 pupper_onnx_node.py --engage --switch --teleop
python3 pupper_onnx_node.py --engage --switch --joy \\
        --joy-engage-button 3

if projected_gravity.z > -0.5:
    trigger e-stop""",
        (153, 128, 255),
    ),
    Scene(
        "14_sim2real_video",
        "Sim2Real 實體 Pupper 行走",
        "這是修正後的實機步態。同一個 ONNX，前進、側移、轉向都自然，高速也不再亂抖。",
        (
            "Raspberry Pi ONNX Runtime",
            "ROS 2 forward position / kp / kd",
            "Forward / side / yaw all natural",
        ),
        "pupper_sim2real_deploy.mp4",
        "",
        (0, 194, 255),
        default_clip=str(ISAAC_LAB / "pupper_sim2real_deploy.mp4"),
    ),
    Scene(
        "15_summary",
        "Summary",
        "總結，從 MuJoCo 到 Isaac Lab，重點是讓訓練、匯出和部署三邊契約一致："
        "固定觀測和動作、對齊 PD 參數、補上延遲和增益隨機化，最後驗證關節順序。"
        "做到這些，Pupper 就能從 Isaac Lab 走到真機。"
        "若要詳細了解，可以點擊下方的 GitHub 連結看完整說明。感謝觀看。",
        (
            "Contract first: obs, action, gains, freq",
            "Sim2real: latency + gain randomization",
            "Deploy: ONNX + ROS 2 + joint-order check",
        ),
        "MuJoCo MJX -> Isaac Lab -> Real Pupper",
        """[1] Fix the policy contract (obs / action / gains)
[2] Align PhysX ImplicitActuator with MJX PD
[3] Add 1-step latency + gain randomization
[4] Export ONNX (normalizer embedded)
[5] Verify joint order on deploy -> walk!""",
        (0, 190, 180),
        link="https://github.com/tommywu052/pupperv3-monorepo",
    ),
]


def run(cmd: list[str], cwd: Path | None = None) -> None:
    print("+", " ".join(str(c) for c in cmd))
    subprocess.run(cmd, cwd=cwd, check=True)


def ffprobe_duration(path: Path) -> float:
    result = subprocess.check_output(
        [
            "ffprobe", "-v", "error", "-show_entries", "format=duration",
            "-of", "default=noprint_wrappers=1:nokey=1", str(path),
        ],
        text=True,
    )
    return float(result.strip())


def load_font(name: str, size: int) -> ImageFont.FreeTypeFont:
    candidates = [
        Path("C:/Windows/Fonts/msjh.ttc"),
        Path("C:/Windows/Fonts/msyh.ttc"),
        Path("C:/Windows/Fonts/NotoSansCJK-Regular.ttc"),
    ]
    if name == "mono":
        candidates = [Path("C:/Windows/Fonts/consola.ttf"), *candidates]
    for path in candidates:
        if path.exists():
            return ImageFont.truetype(str(path), size=size)
    return ImageFont.load_default(size=size)


FONT_TITLE = load_font("sans", 64)
FONT_SUBTITLE = load_font("sans", 34)
FONT_BODY = load_font("sans", 34)
FONT_SMALL = load_font("sans", 26)
FONT_CODE = load_font("mono", 31)


def wrap_text(draw: ImageDraw.ImageDraw, text: str, font: ImageFont.ImageFont, width: int) -> list[str]:
    lines: list[str] = []
    for raw in text.splitlines():
        if not raw:
            lines.append("")
            continue
        line = ""
        for ch in raw:
            test = line + ch
            if draw.textlength(test, font=font) <= width:
                line = test
            else:
                if line:
                    lines.append(line)
                line = ch
        if line:
            lines.append(line)
    return lines


def wrap_code(draw: ImageDraw.ImageDraw, text: str, font: ImageFont.ImageFont, width: int) -> list[str]:
    out: list[str] = []
    for line in text.strip("\n").splitlines():
        expanded = line.expandtabs(4)
        if draw.textlength(expanded, font=font) <= width:
            out.append(expanded)
            continue
        chunk = ""
        for token in re.split(r"(\s+)", expanded):
            if draw.textlength(chunk + token, font=font) <= width:
                chunk += token
            else:
                if chunk:
                    out.append(chunk)
                chunk = "    " + token.lstrip()
        if chunk:
            out.append(chunk)
    return out


def draw_gradient(img: Image.Image, accent: tuple[int, int, int]) -> None:
    pix = img.load()
    base = (13, 18, 25)
    for y in range(H):
        t = y / H
        for x in range(W):
            s = (x / W) * 0.25 + t * 0.25
            pix[x, y] = tuple(int(base[i] * (1 - s) + accent[i] * s * 0.22) for i in range(3))


def render_scene(scene: Scene, path: Path) -> None:
    img = Image.new("RGB", (W, H), (13, 18, 25))
    draw_gradient(img, scene.accent)
    draw = ImageDraw.Draw(img)
    accent = scene.accent
    white = (239, 244, 249)
    muted = (164, 178, 194)
    panel = (22, 29, 39)
    code_bg = (8, 12, 18)

    draw.rectangle((0, 0, W, 10), fill=accent)
    draw.text((90, 74), scene.title, font=FONT_TITLE, fill=white)
    draw.text((92, 154), "Pupper v3 | MJX -> Isaac Lab -> Sim2Real", font=FONT_SUBTITLE, fill=muted)

    left_x, left_y = 92, 250
    draw.rounded_rectangle((70, 220, 790, 930), radius=22, fill=panel, outline=(44, 55, 70), width=2)
    draw.text((left_x, left_y), "Key points", font=FONT_SUBTITLE, fill=accent)
    y = left_y + 76
    for bullet in scene.bullets:
        wrapped = wrap_text(draw, bullet, FONT_BODY, 600)
        draw.ellipse((left_x, y + 13, left_x + 14, y + 27), fill=accent)
        for i, line in enumerate(wrapped):
            draw.text((left_x + 34, y + i * 44), line, font=FONT_BODY, fill=white)
        y += max(1, len(wrapped)) * 46 + 22

    # VS Code-like embedded editor: keeps the video self-contained and makes the
    # code/config sections look like real file walkthroughs.
    win = (820, 220, 1848, 930)
    draw.rounded_rectangle(win, radius=12, fill=(30, 30, 30), outline=(58, 68, 82), width=2)
    draw.rectangle((821, 221, 1847, 265), fill=(36, 36, 36))
    for i, color in enumerate(((255, 95, 86), (255, 189, 46), (39, 201, 63))):
        cx = 850 + i * 24
        draw.ellipse((cx, 236, cx + 12, 248), fill=color)
    draw.text((925, 230), scene.code_title or "README.md", font=FONT_SMALL, fill=(205, 213, 224))

    draw.rectangle((821, 266, 875, 929), fill=(45, 45, 48))
    for y_icon, label in ((300, "F"), (356, "S"), (412, "G"), (468, "R")):
        draw.rounded_rectangle((838, y_icon, 858, y_icon + 26), radius=4, outline=(126, 139, 153), width=2)
        draw.text((842, y_icon + 1), label, font=FONT_SMALL, fill=(190, 199, 210))
    draw.rectangle((876, 266, 1116, 929), fill=(37, 37, 38))
    draw.text((898, 294), "EXPLORER  ai/isaac_lab", font=FONT_SMALL, fill=(176, 186, 198))
    explorer = [
        "README.md",
        "SIM2REAL.md",
        "pupper_isaaclab",
        "  assets/pupper.py",
        "  tasks/locomotion",
        "    pupper_env_cfg.py",
        "    delayed_action.py",
        "    agents",
        "scripts",
        "deploy",
    ]
    ey = 332
    active_name = (scene.code_title or "").split("/")[-1].split(" ")[0]
    for item in explorer:
        fill = (214, 222, 233)
        if active_name and active_name in item:
            draw.rectangle((888, ey - 4, 1104, ey + 28), fill=(55, 71, 89))
            fill = white
        draw.text((898, ey), item, font=FONT_SMALL, fill=fill)
        ey += 32

    editor_x = 1117
    draw.rectangle((editor_x, 266, 1847, 310), fill=(31, 31, 31))
    tab_name = scene.code_title or "README.md"
    short_tab = tab_name.split("/")[-1]
    draw.rectangle((editor_x, 266, min(1847, editor_x + 320), 310), fill=(43, 43, 43))
    draw.text((editor_x + 22, 276), short_tab, font=FONT_SMALL, fill=white)
    draw.rectangle((editor_x, 310, 1847, 346), fill=(24, 24, 24))
    draw.text((editor_x + 22, 318), tab_name, font=FONT_SMALL, fill=muted)

    draw.rectangle((editor_x, 346, 1847, 929), fill=code_bg)
    code_lines = wrap_code(draw, scene.code or scene.narration or scene.title, FONT_CODE, 610)
    y = 372
    for idx, line in enumerate(code_lines[:13], start=1):
        draw.text((editor_x + 24, y), f"{idx:>2}", font=FONT_CODE, fill=(98, 114, 132))
        color = white
        if any(k in line for k in ("target", "delay_prob", "PERM", "INV", "KP", "KD", "normalizer", "stiffness", "damping")):
            color = accent
        elif line.strip().startswith("#"):
            color = (112, 172, 111)
        draw.text((editor_x + 82, y), line, font=FONT_CODE, fill=color)
        y += 40

    img.save(path, quality=95)


# --------------------------------------------------------------------------
# Slide deck -> frames (via pptxgenjs + LibreOffice + PyMuPDF).
# --------------------------------------------------------------------------

SOFFICE = Path(r"C:\Program Files\LibreOffice\program\soffice.exe")


def _hex(rgb: tuple[int, int, int]) -> str:
    return "%02X%02X%02X" % rgb


def write_scenes_json(scenes: list[Scene], path: Path) -> None:
    data = []
    for s in scenes:
        data.append({
            "scene_id": s.scene_id,
            "title": s.title,
            "subtitle": s.code_title,
            "bullets": list(s.bullets),
            "code": s.code,
            "code_title": s.code_title.split("  ")[0] if s.code_title else "config",
            "accent": _hex(s.accent),
            "cover": s.cover,
            "cover_lines": list(s.cover_lines),
            "screenshot": s.screenshot if s.screenshot and Path(s.screenshot).exists() else "",
            "extra_image": s.extra_image if s.extra_image and Path(s.extra_image).exists() else "",
            "link": s.link,
        })
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def build_pptx_frames(scenes: list[Scene], frames_dir: Path) -> None:
    """Render the slide deck to one 1920x1080 PNG per scene (in scene order)."""
    import fitz  # PyMuPDF

    write_scenes_json(scenes, OUT / "scenes.json")

    node = shutil.which("node") or "node"
    run([node, str(ROOT / "build_deck.js")])

    deck = OUT / "deck.pptx"
    if not deck.exists():
        raise RuntimeError("deck.pptx was not produced by build_deck.js")

    if not SOFFICE.exists():
        raise RuntimeError(f"LibreOffice not found at {SOFFICE}")
    profile = (ROOT / ".lo_profile").resolve().as_uri()
    run([
        str(SOFFICE), "--headless", "--norestore",
        f"-env:UserInstallation={profile}",
        "--convert-to", "pdf", "--outdir", str(OUT), str(deck),
    ])
    pdf = OUT / "deck.pdf"
    if not pdf.exists():
        raise RuntimeError("LibreOffice did not produce deck.pdf")

    doc = fitz.open(str(pdf))
    if doc.page_count != len(scenes):
        print(f"[warn] pdf pages {doc.page_count} != scenes {len(scenes)}")
    frames_dir.mkdir(parents=True, exist_ok=True)
    for i, scene in enumerate(scenes):
        page = doc.load_page(min(i, doc.page_count - 1))
        zoom = W / page.rect.width
        pix = page.get_pixmap(matrix=fitz.Matrix(zoom, zoom), alpha=False)
        out_png = frames_dir / f"{scene.scene_id}.png"
        pix.save(str(out_png))
        # PyMuPDF may produce e.g. 1920x1079 depending on rounding; pad/scale in ffmpeg later.
    doc.close()
    print(f"[deck] rendered {len(scenes)} slides -> {frames_dir}")


# --------------------------------------------------------------------------
# TTS backends.
# --------------------------------------------------------------------------

def _load_opentalking_env() -> None:
    src = OPENTALKING / "src"
    if str(src) not in sys.path:
        sys.path.insert(0, str(src))
    env_file = OPENTALKING / ".env"
    if env_file.exists():
        for line in env_file.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if "=" in line and not line.startswith("#"):
                key, val = line.split("=", 1)
                key, val = key.strip(), val.strip()
                if key in (
                    "DASHSCOPE_API_KEY",
                    "OPENTALKING_QWEN_TTS_WS_URL",
                    "DASHSCOPE_WORKSPACE_ID",
                ):
                    os.environ[key] = val


def _write_wav(pcm: np.ndarray, path: Path, sr: int = TTS_SR) -> None:
    with wave.open(str(path), "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sr)
        wf.writeframes(pcm.astype(np.int16).tobytes())


async def synth_qwen(scenes: list[Scene], audio_paths: dict[str, Path]) -> None:
    """Synthesize every scene narration with the cloned Tommy voice (reused WS)."""
    _load_opentalking_env()
    from opentalking.tts.dashscope_qwen.adapter import DashScopeQwenTTSAdapter

    tts = DashScopeQwenTTSAdapter(default_voice=TOMMY_VOICE, model=TOMMY_MODEL, sample_rate=TTS_SR)
    try:
        for scene in scenes:
            if not scene.narration.strip():
                continue
            print(f"[tts:qwen] {scene.scene_id}")
            chunks: list[np.ndarray] = []
            async for chunk in tts.synthesize_stream(scene.narration):
                chunks.append(chunk.data)
            pcm = np.concatenate(chunks) if chunks else np.zeros(0, dtype=np.int16)
            _write_wav(pcm, audio_paths[scene.scene_id])
    finally:
        await tts.aclose()


async def synth_edge(scenes: list[Scene], audio_paths: dict[str, Path]) -> None:
    import edge_tts

    for scene in scenes:
        if not scene.narration.strip():
            continue
        print(f"[tts:edge] {scene.scene_id}")
        communicate = edge_tts.Communicate(scene.narration, voice=EDGE_VOICE, rate=EDGE_RATE)
        await communicate.save(str(audio_paths[scene.scene_id]))


# --------------------------------------------------------------------------
# Subtitles.
# --------------------------------------------------------------------------

def srt_time(seconds: float) -> str:
    ms = int(round(seconds * 1000))
    h, rem = divmod(ms, 3600_000)
    m, rem = divmod(rem, 60_000)
    s, ms = divmod(rem, 1000)
    return f"{h:02d}:{m:02d}:{s:02d},{ms:03d}"


def split_subtitle(text: str, max_len: int = 30) -> list[str]:
    chunks: list[str] = []
    buf = ""
    for ch in text:
        if ch in "，。；：、" and len(buf) >= 10:
            chunks.append(buf + ch)
            buf = ""
        else:
            buf += ch
        if len(buf) >= max_len:
            chunks.append(buf)
            buf = ""
    if buf:
        chunks.append(buf)
    return chunks


def write_srt(durations: dict[str, float], path: Path) -> None:
    idx = 1
    t = 0.0
    lines: list[str] = []
    for scene in SCENES:
        dur = durations[scene.scene_id]
        chunks = split_subtitle(scene.narration)
        if not chunks:
            t += dur
            continue
        step = dur / max(1, len(chunks))
        for chunk in chunks:
            lines.append(str(idx))
            lines.append(f"{srt_time(t)} --> {srt_time(min(t + step, t + dur))}")
            lines.append(chunk)
            lines.append("")
            idx += 1
            t += step
    path.write_text("\n".join(lines), encoding="utf-8")


# --------------------------------------------------------------------------
# Segment builders.
# --------------------------------------------------------------------------

_VF = (
    f"scale={W}:{H}:force_original_aspect_ratio=decrease,"
    f"pad={W}:{H}:(ow-iw)/2:(oh-ih)/2,format=yuv420p"
)


def make_static_segment(image: Path, audio: Path, duration: float, out: Path) -> None:
    run([
        "ffmpeg", "-y",
        "-loop", "1", "-t", f"{duration:.3f}", "-i", str(image),
        "-i", str(audio),
        "-vf", _VF, "-r", str(FPS),
        "-af", f"apad,atrim=0:{duration:.3f}",
        "-c:v", "libx264", "-preset", "veryfast", "-crf", "18",
        "-c:a", "aac", "-b:a", "192k",
        "-t", f"{duration:.3f}",
        str(out),
    ])


def make_clip_segment(clip: Path, audio: Path, duration: float, out: Path) -> None:
    """Loop a (short) demo clip under the narration, drop source audio."""
    run([
        "ffmpeg", "-y",
        "-stream_loop", "-1", "-i", str(clip),
        "-i", str(audio),
        "-t", f"{duration:.3f}",
        "-vf", _VF, "-r", str(FPS),
        "-af", f"apad,atrim=0:{duration:.3f}",
        "-map", "0:v:0", "-map", "1:a:0",
        "-c:v", "libx264", "-preset", "veryfast", "-crf", "18",
        "-c:a", "aac", "-b:a", "192k",
        str(out),
    ])


def normalize_segment(src: Path, dst: Path) -> None:
    """Normalize timing/audio params so concat is stable across all segments."""
    run([
        "ffmpeg", "-y", "-fflags", "+genpts", "-i", str(src),
        "-vf", f"setpts=PTS-STARTPTS,scale={W}:{H},format=yuv420p",
        "-af", "aresample=48000:async=1:first_pts=0",
        "-ac", "2", "-ar", "48000", "-r", str(FPS),
        "-c:v", "libx264", "-preset", "veryfast", "-crf", "18",
        "-c:a", "aac", "-b:a", "192k",
        str(dst),
    ])


def burn_subtitles(src: Path, srt: Path, dst: Path) -> None:
    run([
        "ffmpeg", "-y", "-i", str(src),
        "-vf", (
            "subtitles=" + srt.name
            + ":force_style='FontName=Microsoft JhengHei,FontSize=22,"
            + "PrimaryColour=&H00FFFFFF&,OutlineColour=&HAA000000&,"
            + "BorderStyle=1,Outline=2,Shadow=1,Alignment=2,MarginV=44'"
        ),
        "-c:v", "libx264", "-preset", "veryfast", "-crf", "18",
        "-c:a", "copy", str(dst),
    ], cwd=srt.parent)


def parse_clips(values: Iterable[str]) -> dict[str, Path]:
    clips: dict[str, Path] = {}
    for value in values:
        if "=" not in value:
            raise ValueError("--clip must look like scene_id=C:/path/demo.mp4")
        key, raw = value.split("=", 1)
        clips[key] = Path(raw).expanduser().resolve()
    return clips


def resolve_clip(scene: Scene, clips: dict[str, Path]) -> Path | None:
    if scene.scene_id in clips:
        return clips[scene.scene_id]
    if scene.default_clip:
        path = Path(scene.default_clip)
        if path.exists():
            return path
    return None


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tts", choices=["qwen", "edge"], default="qwen",
                        help="qwen = cloned Tommy voice (default); edge = EdgeTTS")
    parser.add_argument("--slides", choices=["pptx", "pil"], default="pptx",
                        help="pptx = slide deck via pptxgenjs+LibreOffice (default); pil = legacy PIL frames")
    parser.add_argument("--clip", action="append", default=[], help="scene_id=C:/path/demo.mp4")
    parser.add_argument("--clip-pad", type=float, default=0.6,
                        help="extra seconds so the demo clip can finish under the narration")
    parser.add_argument("--keep", action="store_true", help="Do not clear out/")
    args = parser.parse_args()

    if not shutil.which("ffmpeg") or not shutil.which("ffprobe"):
        raise RuntimeError("ffmpeg/ffprobe must be on PATH")

    clips = parse_clips(args.clip)
    if OUT.exists() and not args.keep:
        shutil.rmtree(OUT)
    audio_ext = "wav" if args.tts == "qwen" else "mp3"
    (OUT / "audio").mkdir(parents=True, exist_ok=True)
    (OUT / "frames").mkdir(parents=True, exist_ok=True)
    (OUT / "segments").mkdir(parents=True, exist_ok=True)

    # 1. Render frames: slide deck (default) or legacy PIL frames.
    frames_dir = OUT / "frames"
    if args.slides == "pptx":
        build_pptx_frames(SCENES, frames_dir)
    else:
        for scene in SCENES:
            print(f"[render] {scene.scene_id}")
            render_scene(scene, frames_dir / f"{scene.scene_id}.png")

    # 2. Generate narration audio (single event loop -> reuse TTS websocket).
    audio_paths = {s.scene_id: OUT / "audio" / f"{s.scene_id}.{audio_ext}" for s in SCENES}
    if args.tts == "qwen":
        asyncio.run(synth_qwen(SCENES, audio_paths))
    else:
        asyncio.run(synth_edge(SCENES, audio_paths))

    # 3. Build one segment per scene.
    manifest: dict[str, object] = {"tts": args.tts, "voice": TOMMY_VOICE if args.tts == "qwen" else EDGE_VOICE, "scenes": []}
    durations: dict[str, float] = {}
    segment_paths: list[Path] = []

    for scene in SCENES:
        frame = OUT / "frames" / f"{scene.scene_id}.png"
        audio = audio_paths[scene.scene_id]
        segment = OUT / "segments" / f"{scene.scene_id}.mp4"
        clip = resolve_clip(scene, clips)

        narration_dur = ffprobe_duration(audio) if audio.exists() else 0.0

        if clip is not None:
            clip_len = ffprobe_duration(clip)
            duration = max(narration_dur, clip_len) + args.clip_pad
            print(f"[clip] {scene.scene_id} <- {clip.name} (clip {clip_len:.1f}s, narr {narration_dur:.1f}s)")
            make_clip_segment(clip, audio, duration, segment)
            visual = str(clip)
        else:
            duration = narration_dur + 0.3
            make_static_segment(frame, audio, duration, segment)
            visual = str(frame)

        durations[scene.scene_id] = duration
        segment_paths.append(segment)
        manifest["scenes"].append({
            "id": scene.scene_id,
            "title": scene.title,
            "duration": round(duration, 3),
            "visual": visual,
            "audio": str(audio),
        })

    # 4. Subtitles + normalize + concat.
    srt = OUT / "pupper_mjx_to_isaac_lab.srt"
    write_srt(durations, srt)

    norm_dir = OUT / "segments_norm"
    norm_dir.mkdir(parents=True, exist_ok=True)
    norm_paths: list[Path] = []
    for segment in segment_paths:
        norm = norm_dir / segment.name
        normalize_segment(segment, norm)
        norm_paths.append(norm)

    concat = OUT / "concat_norm.txt"
    concat.write_text("".join(f"file '{p.as_posix()}'\n" for p in norm_paths), encoding="utf-8")

    final_mp4 = OUT / "pupper_mjx_to_isaac_lab_5min_release.mp4"
    run(["ffmpeg", "-y", "-f", "concat", "-safe", "0", "-i", str(concat), "-c", "copy", str(final_mp4)])

    subtitled_mp4 = OUT / "pupper_mjx_to_isaac_lab_5min_subtitled.mp4"
    burn_subtitles(final_mp4, srt, subtitled_mp4)

    total = sum(durations.values())
    manifest["total_duration"] = round(total, 3)
    manifest["video"] = str(final_mp4)
    manifest["video_subtitled"] = str(subtitled_mp4)
    manifest["subtitles"] = str(srt)
    (OUT / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")

    minutes = math.floor(total / 60)
    seconds = total - minutes * 60
    print(f"\nDONE: {final_mp4}")
    print(f"Duration: {minutes}:{seconds:04.1f}")
    print(f"Subtitled: {subtitled_mp4}")
    print(f"Subtitles: {srt}")


if __name__ == "__main__":
    main()
