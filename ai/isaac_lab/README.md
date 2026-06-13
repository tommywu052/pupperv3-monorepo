# Pupper v3 — Isaac Lab 強化學習移植 + sim2real

> 語言：**繁體中文** | [English](README.en.md)

把原本在 **MJX (MuJoCo XLA) + Brax PPO** 訓練的 Pupper v3 行走 policy，移植到
**NVIDIA Isaac Lab (PhysX GPU 仿真 + rsl_rl PPO)** 訓練，並一路打通到實機
（Raspberry Pi，onnxruntime 推論）直接遙控行走。

- **訓練**：Windows 原生 Isaac Lab v2.3.2 + Isaac Sim 5.1，RTX Pro 6000 (Blackwell, 96GB)。
- **觀測/動作契約（方案 B）**：採 Isaac Lab 標準 locomotion 契約。為了能部署到實機/Jetson，
  使用**不對稱 actor-critic**：actor 只用實機可量測的量（IMU 角速度、投影重力、命令、
  關節角/速、上一步 action），critic 可另含 privileged 量（base 線速度、rough 地形高度圖）。
- **部署（方案 2）**：訓練產物 → **ONNX**，在 Raspberry Pi（onnxruntime CPU，實測 50 Hz 綽綽有餘）
  即時推論，作為 C++ `neural_controller`（RTNeural）的替代；Jetson NX 才需 TensorRT。

最終實機**前後左右皆自然行走、可鍵盤/搖桿遙控**。全程未改網路結構——最終那版 `policy.onnx`
一字未改，只修部署端排列就會走。

---

## 1. 契約（訓練 / 匯出 / 部署三邊必須一致）

採 **Isaac Lab 標準 locomotion 契約（方案 B）**，單一 frame、45 維觀測：

```
obs(45) = base_ang_vel(3)
        + projected_gravity(3)
        + velocity_commands(3)          # vx, vy, wz
        + (joint_pos - default)(12)     # policy 關節順序
        + joint_vel(12)                 # policy 關節順序
        + last_action(12)               # 上一步原始網路輸出
action(12) = 原始網路輸出
target = default_joint_pos + 0.25 * action      (clip 到關節極限)
PD gains : KP = 5.0,  KD = 0.25      (force-mode PD，對齊 MuJoCo)
control  : 50 Hz       physics : 200 Hz
projected_gravity = quat_rotate_inverse(imu_orientation, [0,0,-1])
```

數值（站姿 / 極限 / gains）單一真值來源：`pupper_isaaclab/assets/pupper.py`。
匯出時**只匯 actor**，critic 的 privileged 觀測不影響部署。

---

## 2. 訓練配方：把 MJX 的 sim2real recipe 補進 Isaac Lab

純物理對齊（effort/velocity limit、KP/KD）不夠 —— 早期版本在 sim 走得好、實機卻只會
「傾身不踏步」或低速凍結、高速亂抖。缺的是 MJX 用來跨越 sim2real gap 的 DR，缺一不可：

| 機制 | 設定 | 作用 |
|------|------|------|
| **ImplicitActuator** | PhysX PD 跑在 200 Hz physics rate | 對應實機 ~520 Hz 內部 PD。**不要**用 `DelayedPDActuator`（PD 只跑 50 Hz 會振鈴、`dof_acc` 爆掉）|
| **1-step action latency** | `DelayedJointPositionAction`，`delay_prob=0.8` | 模擬實機致動延遲；讓步態在實機**活起來**的關鍵 |
| **actuator gain randomization** | `randomize_actuator_gains`，stiffness ×[0.6,1.1]、damping ×[0.8,1.5]，`mode="startup"` | 對應 MJX `kp_mult/kd_mult`；讓步態**穩健**、不會欠阻尼翻倒 |
| 其他 | friction、base mass/COM jitter、kick events | 一般 locomotion DR |

> 經驗：只有延遲 → 會踏步但欠阻尼會翻；只有 gain rand → 凍結。**延遲 + gain rand** 一起才穩。
> 關鍵實作見 `tasks/locomotion/delayed_action.py` 與 `tasks/locomotion/pupper_env_cfg.py`。

**姿勢 / 防抖獎勵**（rough 版）：基礎獎勵集沒有站高/姿態項，軟 KP 下 policy 會學「壓低重心」
而趴著走。已加 `base_height_l2`（絕對目標 0.14m）、`joint_deviation_hip`、`stand_still`
（零指令時釘回站姿）、提高 `action_rate` 懲罰與站定環境比例來修正。
> ⚠️ `base_height_l2` 刻意用**絕對**目標、避免地形相對（`sensor_cfg`）——後者在射線打不到地形時會
> 爆量、把 PPO 的動作噪聲 std 推成負值（`RuntimeError: normal expects std >= 0.0`）而炸掉訓練。

**任務**：`Pupper-Flat-v0`（平地）、`Pupper-Rough-v0`（不平/階梯/斜坡，critic 多吃地形高度圖）。
平地約 600 iters；rough 視課程進展，數千 iters。

---

## 3. 匯出

`scripts/play.py`（改編自 Isaac Lab rsl_rl play）載入 checkpoint 後，用
`export_policy_as_onnx(policy_nn, normalizer=...)` 匯出。**observation normalizer
（每維 running mean/std）被內建進 ONNX**，部署端餵**原始** observation 即可（不要自己再 normalize）。
產物：`exported/policy.onnx`（+ `policy.pt`）。

> 因為 normalizer 是 per-element 的，**observation 每一維順序都必須完全正確**，否則每個值會套到
> 錯誤的統計 → 見第 5 節的關節順序坑。

---

## 4. 實機部署（Raspberry Pi，onnxruntime）

`deploy/pupper_onnx_node.py` 用 onnxruntime + rclpy 直接在 Pi（aarch64 / ROS 2 jazzy）推論，
**不改任何 C++**，沿用 `robot.service` 既有的 ros2_control 介面：

| 方向 | Topic | 用途 |
|------|-------|------|
| sub | `/joint_states` | 關節角/速（名稱順序亂，節點依名稱重排成**硬體序**再轉 policy 序）|
| sub | `/imu_sensor_broadcaster/imu` | body-frame 角速度 + orientation（算 projected gravity）|
| sub | `/cmd_vel` 或 `/joy` | 速度命令 |
| pub | `/forward_position_controller/commands` | 關節位置目標 |
| pub | `/forward_kp_controller/commands` / `/forward_kd_controller/commands` | kp=5.0 / kd=0.25 |

`forward_*` controllers 預設 inactive；engage 時節點呼叫 `/controller_manager/switch_controller`
把 `neural_controller` 換成這三個 forward controllers。

**上傳 + 執行**（Pi 上先 `source /opt/ros/jazzy/setup.bash` 與 ros2_ws overlay、`export ROS_LOCALHOST_ONLY=1`）：

```bash
# 從開發機上傳（scp）
scp <exported>/policy.onnx        pi@<PI_IP>:/home/pi/pupper_policy/policy.onnx
scp deploy/pupper_onnx_node.py    pi@<PI_IP>:/home/pi/pupper_policy/pupper_onnx_node.py

# Pi 上：dry-run（只讀+推論不送馬達，安全）
python3 ~/pupper_policy/pupper_onnx_node.py --duration 12
# engage（實際驅動馬達，務必架高/淨空）
python3 ~/pupper_policy/pupper_onnx_node.py --engage --switch
# 搖桿遙控（方塊=啟動站立+驅動，搖桿軸移動）
python3 ~/pupper_policy/pupper_onnx_node.py --engage --switch --joy --joy-engage-button 3
```

**安全機制**：`--switch`（切到 `forward_*`）、init ramp → fade-in（平滑回站姿再混入 policy）、
傾倒 e-stop（`projected_gravity.z > -0.5` 即鎖 default 姿態並提高 kd）、退出回站姿。
開機自動啟動可用 `deploy/pupper_policy.service` + `deploy/pupper_policy.sh`（systemd）。

**搖桿與 e-stop**：PS 佈局 `X=0`、`O=1` 已被 `estop_controller` 佔用，故用**方塊(=3)**啟動本 policy。
**R3（button 12）= e-stop**：節點訂閱 `/emergency_stop`，搶在控制器被停用**前**寫入 `kp=0、kd=estop_kd(0.1)`
讓腿**變軟**（與 RTNeural `on_deactivate` 一致），而非僵硬定住；**e-stop 後再按方塊**即可重新站起。
軸對應（同 `teleop_twist_joy`）：左搖桿上下=`vx`(×0.75)、左右=`vy`(×0.5)、右搖桿左右=`wz`(×2.0)。

---

## 5. 根因坑：Isaac Lab 關節順序錯位（sim2real 最大兇手）

**症狀**：sim（play）走得好；實機站得住、會踏步，但前進變原地滑/轉、yaw 幾乎沒反應，高速亂抖。

**根因**：Isaac Lab 與硬體的 12 個關節排序不同——
- **硬體 / MJCF / `neural_controller` config**：依**腿**分組 `FR_1,FR_2,FR_3, FL..., BR..., BL...`
- **Isaac Lab（URDF→USD 後 PhysX 的 DOF 序）**：依運動鏈**層級**分組 `_1×4 → _2×4 → _3×4`
  （層內 `back_l, back_r, front_l, front_r`）

Policy 在 play 裡 obs/action 都用 Isaac Lab 序（自洽，所以正常）；但部署節點原本假設硬體序就是
policy 序 → obs 的 `joint_pos`/`joint_vel` 與 action 被**沉默地洗牌**。同層關節幅度相近，所以站得住、
會踏步，但每條腿的動作被換到別條腿 → 方向全亂。

**修正（只動部署端，免重訓、免重匯出）**：讀 `/joint_states`、發送馬達命令**維持硬體序**，但
**組 obs、解讀 action 改用 policy 序**：

```
q_pol      = q_hw[PERM]                       # 餵網路前轉 policy 序
target_pol = default_pol + 0.25 * action      # action 是 policy 序
target_hw  = target_pol[INV]                  # 送 forward_position_controller 前轉回硬體序
PERM = [9, 6, 3, 0, 10, 7, 4, 1, 11, 8, 5, 2]   # 硬體序 → policy 序
INV  = [3, 7, 11, 2, 6, 10, 1, 5, 9, 0, 4, 8]   # policy 序 → 硬體序
```

驗證工具：`scripts/check_contract.py`（載入真正的 articulation，印出 `robot.joint_names`、
action/obs layout，並自動算出對照排列）。

**為什麼 MJX 沒事**：MuJoCo 全程沿用 MJCF 宣告順序，訓練 obs/action 與部署 config 同一份「依腿分組」
順序，天生對齊；Isaac Lab 換了仿真器、PhysX 重排了 DOF，而契約沒同步。
> 教訓：**換仿真器 = 關節順序契約必須重新驗證**，別假設順序，一律查 `robot.joint_names`。

---

## 6. 目錄結構

```
ai/isaac_lab/
├── pupper_isaaclab/
│   ├── assets/pupper.py            # ArticulationCfg + 部署契約 (站姿/極限/PD gains/USD)
│   └── tasks/locomotion/
│       ├── pupper_env_cfg.py       # env cfg (action latency + gain rand + 姿勢獎勵 + 地形)
│       ├── delayed_action.py       # DelayedJointPositionAction (sim2real 關鍵)
│       ├── rewards.py              # 自訂獎勵 (foot_clearance 等)
│       └── agents/                 # rsl_rl PPO cfg
├── scripts/
│   ├── convert_pupper_urdf.py      # URDF(去 world link) → USD
│   ├── train.py / play.py          # 訓練 / play+匯出 ONNX
│   ├── check_contract.py           # 印出 Isaac Lab 實際關節順序並比對 (排錯必跑)
│   └── verify_onnx.py              # 驗證匯出的 ONNX I/O
├── deploy/                         # Raspberry Pi onnxruntime 推論節點 + systemd
│   ├── pupper_onnx_node.py
│   ├── pupper_policy.service / pupper_policy.sh
│   └── diag_cmd_response.py
└── pyproject.toml
```

---

## 7. 端到端重現步驟

```powershell
cd C:\Nvidia\IsaacLab\IsaacLab
# (1) URDF → USD
.\isaaclab.bat -p ...\ai\isaac_lab\scripts\convert_pupper_urdf.py --headless
# (2) 安裝本套件
.\isaaclab.bat -p -m pip install -e C:\Nvidia\pupperv3\pupperv3-monorepo\ai\isaac_lab
# (3) 訓練（平地 ~600 iters；rough 用 Pupper-Rough-v0）
.\isaaclab.bat -p ...\ai\isaac_lab\scripts\train.py --task Pupper-Flat-v0 --headless
# (4) play + 匯出 ONNX（normalizer 內建進 ONNX）
.\isaaclab.bat -p ...\ai\isaac_lab\scripts\play.py --task Pupper-Flat-Play-v0 --num_envs 32
# (5) 驗證關節/觀測契約（換仿真器或改 URDF 後務必跑）
.\isaaclab.bat -p ...\ai\isaac_lab\scripts\check_contract.py --task Pupper-Flat-Play-v0 --headless
```

```bash
# (6) 上傳到 Pi 並執行（見第 4 節）
scp <exported>/policy.onnx pi@<PI_IP>:/home/pi/pupper_policy/policy.onnx
python3 ~/pupper_policy/pupper_onnx_node.py --duration 12                                   # dry-run
python3 ~/pupper_policy/pupper_onnx_node.py --engage --switch --joy --joy-engage-button 3   # 搖桿（方塊啟動）
```

---

## 8. Checklist / 經驗

- [ ] **關節順序**：換仿真器/改 URDF 後，先 `check_contract.py` 對齊（最易踩、最沉默）。
- [ ] **observation 順序與 normalizer**：ONNX 內建 per-element normalizer，obs 每維順序都要對。
- [ ] **sim2real DR**：1-step action latency + actuator gain randomization 缺一不可。
- [ ] **actuator 模型**：高 PD-rate 機器人用 ImplicitActuator + action delay，別用 50 Hz 的 DelayedPDActuator。
- [ ] **獎勵數值穩定**：避免無上界的獎勵（如地形相對 `base_height_l2`），否則 std 變負、訓練炸。
- [ ] **IMU frame**：以能正常走的 RTNeural 為真值（`ang_vel` 直接用、`projected_gravity = R(q)^T·[0,0,-1]`）。
- [ ] **安全**：實機務必架高/淨空，先 dry-run，保留傾倒 e-stop 與退出回站姿。
- [ ] **e-stop 放軟**：用 `forward_command_controller` 時標準停用不會放軟；需訂閱 `/emergency_stop`
      在停用前寫 `kp=0/kd=estop_kd`，並讓啟動鍵能重新 `switch` 恢復。
