# Pi overlay workspaces

Additional ROS 2 colcon workspaces on the Raspberry Pi **outside** `pupperv3-monorepo/ros2_ws`.

Pi 上除了 monorepo 的 `ros2_ws` 外，還有兩個 **overlay workspace**（路徑在 `/home/pi/`）：

| Pi path | Repo docs | Purpose |
|---------|-----------|---------|
| `/home/pi/nav2_ws` | [nav2_ws/README.md](nav2_ws/README.md) | Nav2 + deps (source build, Jazzy) |
| `/home/pi/ldlidar_ros2_ws` | [ldlidar_ros2_ws/README.md](ldlidar_ros2_ws/README.md) | LD06 LiDAR driver |

Each subdirectory contains a **`*.repos` vcstool manifest** — not the full source tree or `build/`/`install/` artifacts.

Typical Pi shell setup:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ldlidar_ros2_ws/install/setup.bash   # optional, for LiDAR
source ~/nav2_ws/install/setup.bash           # optional, for Nav2
source ~/pupperv3-monorepo/ros2_ws/install/setup.bash
```

Build helpers: `scripts_local/build_nav2_source_pi.py`, `scripts_local/build_slam_jazzy_pi.py`.
