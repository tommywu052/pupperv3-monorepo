# Pi overlay: `~/nav2_ws`

Nav2 and dependencies are **source-built on the Pi** because ROS 2 Jazzy `.deb` packages are unavailable on Raspberry Pi OS (non-Ubuntu Noble).

Pi 上因無 Jazzy apt 套件，Nav2 以 **source overlay workspace** 建置於 `/home/pi/nav2_ws`（注意：不是 `nav_ws`）。

---

## What lives on the Pi / Pi 實際目錄

```text
/home/pi/nav2_ws/
  src/
    navigation2/          # ros-navigation/navigation2 @ jazzy
    bond_core/            # ros/bond_core @ jazzy
    common_interfaces/    # ros2/common_interfaces @ jazzy
    geographic_info/      # ros-geographic-info/geographic_info @ ros2
    BehaviorTree.CPP/     # tag 4.7.3
  build/ install/ log/    # colcon outputs (~780MB total — NOT in git)
```

---

## Reproduce from this repo / 從 repo 重建

1. Use the vcstool manifest:

```bash
mkdir -p ~/nav2_ws/src
cd ~/nav2_ws
vcs import src < pupperv3-monorepo/pi_overlay/nav2_ws/nav2_overlay.repos
```

2. Build with the same flow as `scripts_local/build_nav2_source_pi.py` (subset of Nav2 packages for Pi RAM/CPU).

3. Source before navigation:

```bash
source /opt/ros/jazzy/setup.bash
source ~/nav2_ws/install/setup.bash
source ~/pupperv3-monorepo/ros2_ws/install/setup.bash
```

`scripts_local/pi_start_nav.sh` sources `nav2_ws` automatically when present.

---

## Why not commit `build/` / `install/`?

The compiled overlay is **platform-specific** and large (~780MB). This repo tracks **manifest + docs + build script** only; clone and `colcon build` on the Pi.

編譯產物不納入 git；只保留 manifest 與建置說明，在 Pi 上重建即可。
