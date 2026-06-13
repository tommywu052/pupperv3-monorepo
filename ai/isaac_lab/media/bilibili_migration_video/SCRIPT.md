# 講稿：5 分鐘從 MuJoCo 遷移到 Isaac Lab，並完成 Pupper sim2real

旁白聲音：可用 EdgeTTS（`zh-TW-HsiaoChenNeural`）或 DashScope Qwen 複製的 Tommy 聲音。
無封面首頁，直接從 Agenda 開場。投影片為無品牌（已移除 NVIDIA logo/字樣）白底簡報，
右側嵌入對應的真實 VS Code 截圖或暗色 code/config 面板。

---

## 1. Agenda（開場）— `ai/isaac_lab/README.md`（資料夾總覽截圖）
大家好，這支影片用五分鐘，帶你把 Pupper 從 MuJoCo MJX 遷移到 Isaac Lab，並完成 sim2real。內容分成五段：MJX 參數與 Isaac Lab 設定、遷移架構、Isaac Lab 訓練要點、policy 部署真機要點，最後做總結。

## 2. 遷移架構 — 目錄樹狀圖（`isaac_lab_architecture_tree.png`）
先看架構。訓練從 Brax PPO 換成 rsl_rl PPO，物理從 MJX 換成 PhysX，模型由 URDF 轉成 USD。網路採不對稱 actor-critic，actor 只看真機可量測的觀測，最後匯出 ONNX 到 Raspberry Pi 部署。

## 3. 固定 Policy Contract — `PupperObservationCfg.png`
第一步是固定 policy 契約。Actor 觀測四十五維，只用真機可量測的量：IMU 角速度、投影重力、速度命令、關節角和速度，加上一個 action。動作十二維，乘零點二五加到站姿，就是位置目標。

## 4. PhysX Drive 對齊 MJX PD — `pupper_isaaclab/assets/pupper.py`
在 Isaac Lab，policy 輸出位置目標，由 PhysX 的 ImplicitActuator 產生 torque。stiffness 是 KP 五，damping 是 KD 零點二五，physics 兩百赫茲、控制五十赫茲，對齊 MuJoCo。

## 5. Isaac Lab Env 設定 — `PupperEnvConfig.png`
把 MJX 的命令範圍和時間尺度搬過來：physics 五毫秒、decimation 四，控制五十赫茲，episode 十秒。速度命令涵蓋前後零點七五、側移零點五、yaw 正負二。

## 6. Sim2Real Recipe — `tasks/locomotion/delayed_action.py`
讓實機真正走起來的關鍵，是補回 MJX 的 sim2real 配方：一步 action 延遲，加上致動器增益隨機化。只有延遲容易翻，只有隨機化會凍住，兩個一起才會穩。

## 7. PPO 訓練要點 — `rsl_rl_ppo_pupper.png`
訓練用 rsl_rl PPO，採不對稱 actor-critic。平地約六百個 iteration 就能得到可部署版本，崎嶇地形拉到一千五百。

## 8. 訓練過程縮時（影片）— `videos/pupper_training_progression.mp4`
先看訓練過程。這段縮時可以看到 policy 從一開始亂動、站不穩，逐步學會協調四條腿，最後收斂成穩定步態。

## 9. 訓練成果（影片）— `isaac_rl_result_pupper.mp4`
這是 Isaac Lab 的訓練成果。Policy 穩定追蹤速度命令，步態收斂成乾淨的 limit cycle。

## 10. 匯出 ONNX — `scripts/play.py`
訓練完用 play 腳本匯出 ONNX。關鍵是 normalizer 會包進 ONNX，部署端餵原始觀測就好；也因此，觀測每一維順序都必須正確。

## 11. Raspberry Pi 部署 — `deploy/pupper_onnx_node.py`
真機在 Raspberry Pi 上跑 onnxruntime、用 ROS 2。節點訂閱 joint states、IMU 和 cmd vel，推論後發布到 position、kp、kd 三個 controller，由馬達 PD 產生 torque。

## 12. 最大坑：Joint Order — `deploy/pupper_onnx_node.py`
最大的坑是關節順序。MJX 依腿分組，但 Isaac Lab 經 PhysX 依層級重排。Sim 裡自洽所以正常，一到真機，action 就打到錯的腿。修法是在部署端加入 permutation，免重訓、免重匯出。

## 13. 真機安全與操控 — `deploy/README.md`
部署先 dry run，只讀感測、不送馬達。正式啟動從目前姿態 ramp 到站姿再淡入 policy；偵測翻倒就觸發 e-stop。鍵盤用 WSAD，搖桿用方塊鍵啟動。

## 14. Sim2Real 實機行走（影片）— `pupper_sim2real_deploy.mp4`
這是修正後的實機步態。同一個 ONNX，前進、側移、轉向都自然，高速也不再亂抖。

## 15. 總結 — Summary
總結，從 MuJoCo 到 Isaac Lab，重點是讓訓練、匯出和部署三邊契約一致：固定觀測和動作、對齊 PD 參數、補上延遲和增益隨機化，最後驗證關節順序。做到這些，Pupper 就能從 Isaac Lab 走到真機。感謝觀看。
