# Pupper MJX -> Isaac Lab Bilibili Video

可重現的中文短片產線，主題：把 Pupper 從 MuJoCo / MJX 遷移到 Isaac Lab 並完成
sim2real。成品約 **5 分鐘**，適合上傳 B 站。

目前版本內含：

- **無品牌白底簡報投影片**（本地 `deck_template.js`，已移除 NVIDIA logo/字樣；
  白底、置中標題、綠色檔名副標、頁碼），由 `build_deck.js` 產生 `deck.pptx`，
  再用 LibreOffice + PyMuPDF 轉成 1080p frame。無封面首頁，直接從 Agenda 開場。
- 投影片右側嵌入**真實 VS Code 截圖**：Agenda 用 `isaac-lab-folder-overview.png`、
  架構頁用 `isaac_lab_architecture_tree.png`（目錄樹狀圖）、契約頁用
  `PupperObservationCfg.png`、Env 頁用 `PupperEnvConfig.png`、PPO 頁用
  `rsl_rl_ppo_pupper.png`；其餘章節用暗色 VS Code 風格 code/config 面板。
- 中文旁白：`--tts edge` 用 EdgeTTS（`zh-TW-HsiaoChenNeural`），或 `--tts qwen`
  用 DashScope Qwen 複製的 Tommy 聲音（同 `opentalking/scripts/offline_tts.py`）。
- 三段 demo 影片疊上旁白：訓練過程縮時 `videos/pupper_training_progression.mp4`、
  訓練成果 `isaac_rl_result_pupper.mp4`、實機步態（直式）`pupper_sim2real_deploy.mp4`。
- 外掛 `.srt` 字幕與燒錄字幕版 MP4。

## 產物（out/）

- `pupper_mjx_to_isaac_lab_5min_release.mp4` — 乾淨發布版（無燒錄字幕）。
- `pupper_mjx_to_isaac_lab_5min_subtitled.mp4` — 燒錄中文字幕版。
- `pupper_mjx_to_isaac_lab.srt` — 外掛字幕。
- `deck.pptx` / `deck.pdf` — 簡報投影片原始檔（可再編輯）。
- `scenes.json` — `make_video.py` 給 `build_deck.js` 的場景資料。
- `manifest.json` — 各場景時長、畫面來源、音檔來源。

15 個場景（12 段投影片講解 + 3 段 demo 影片）。EdgeTTS 約 4:23，Tommy 約 5:00。

## 講稿

完整中文講稿見 [`SCRIPT.md`](SCRIPT.md)。議程：MJX 參數與 Isaac Lab 設定 ->
遷移架構 -> Isaac Lab 訓練要點 -> Policy 部署真機要點 -> 總結。

## 重建

```powershell
cd C:\Nvidia\pupperv3\pupperv3-monorepo\ai\isaac_lab\media\bilibili_migration_video
C:\Python312\python.exe .\make_video.py                  # 預設：NVIDIA 投影片 + Tommy 聲音
C:\Python312\python.exe .\make_video.py --slides pil     # 退回舊版 PIL frame
C:\Python312\python.exe .\make_video.py --tts edge       # 退回 EdgeTTS
```

投影片產線（`--slides pptx`，預設）：
1. `make_video.py` 把 15 個場景寫成 `out/scenes.json`；
2. `build_deck.js`（pptxgenjs + `nvidia-pptx` 範本）產生 `out/deck.pptx`；
3. LibreOffice headless 把 `deck.pptx` 轉成 `deck.pdf`；
4. PyMuPDF 把每頁算成 `out/frames/<scene_id>.png`（1920×1080）。

需求：`node` + `pptxgenjs`（在 `~/.cursor/skills/nvidia-pptx`）、
LibreOffice（`C:\Program Files\LibreOffice\program\soffice.exe`）、`pip install pymupdf`。

`--tts qwen`（預設）會：
1. 從 `C:\Nvidia\opentalking\.env` 讀取 `DASHSCOPE_API_KEY` 等變數；
2. 用 voice `qwen-tts-vc-tommyvoice-voice-...`、model `qwen3-tts-vc-realtime-...`
   逐場景合成 16 kHz WAV（單一 event loop 內復用同一條 WebSocket）。

需求：`pip install dashscope`（已裝於 `C:\Python312`）、`ffmpeg`/`ffprobe` 在 PATH、
Windows 中文字型（msjh / msyh）。所有中間段都先正規化成 1080p、30 fps、AAC 48 kHz
立體聲再串接，避免 TTS 投影片與相機/模擬影片混接時的時間戳問題。

## 替換 demo 影片

三段 demo 影片自動偵測：

- `08b_training_progress`: `ai/isaac_lab/videos/pupper_training_progression.mp4`
- `09_training_video`: `ai/isaac_lab/isaac_rl_result_pupper.mp4`
- `14_sim2real_video`: `ai/isaac_lab/pupper_sim2real_deploy.mp4`

用 `--clip scene_id=path.mp4` 覆寫：

```powershell
C:\Python312\python.exe .\make_video.py `
  --clip 09_training_video=C:\Videos\new_isaac_lab_training.mp4 `
  --clip 14_sim2real_video=C:\Videos\new_real_pupper_walk.mp4
```

demo 片段會循環播放原始影片、疊上旁白；片長取「旁白長度」與「影片長度」較大者，
確保短片也能完整播完。
