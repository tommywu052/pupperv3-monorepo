# Pi overlay: `~/ldlidar_ros2_ws`

LD06 LiDAR driver workspace on the Pi. Publishes `/scan` for SLAM and Nav2.

Pi 上的 LD06 雷射驅動 overlay（目錄名為 **`ldlidar_ros2_ws`**，不是 `ldlidar_ros_ws`）。

---

## Layout on Pi

```text
/home/pi/ldlidar_ros2_ws/
  src/ldlidar_stl_ros2/   # github.com/ldrobotSensorTeam/ldlidar_stl_ros2 @ master
  install/                # colcon output (~8MB — NOT in git)
```

---

## Reproduce

```bash
mkdir -p ~/ldlidar_ros2_ws/src
cd ~/ldlidar_ros2_ws
vcs import src < pupperv3-monorepo/pi_overlay/ldlidar_ros2_ws/ldlidar_overlay.repos

source /opt/ros/jazzy/setup.bash
cd ~/ldlidar_ros2_ws
colcon build --symlink-install --packages-select ldlidar_stl_ros2
```

Source chain used by SLAM / Nav2 scripts:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ldlidar_ros2_ws/install/setup.bash
source ~/pupperv3-monorepo/ros2_ws/install/setup.bash
```

`pi_start_nav.sh` can auto-launch `ld06.launch.py` if `/scan` is missing.

---

## Relation to monorepo

Launch files in `pupper_slam` and `pupper_nav` assume this overlay exists on the Pi. The driver source is upstream; we track a **repos manifest** rather than vendoring the full driver tree.
