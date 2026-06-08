# Pupper V3 Codebase

# Deploying to real robot
Follow instructions here https://pupper-v3-documentation.readthedocs.io/en/latest/guide/software_installation.html to flash your Raspberry Pi 5 with our custom image.

# Deploying to simulated robot on development machine (x86 Ubuntu 24)
## Install 
```sh
sudo apt install git-lfs
git lfs install
git clone https://github.com/Nate711/pupperv3-monorepo.git --recurse-submodules
./install_dev_dependencies.sh
```

## Build
```sh
cd ros2_ws
source build.sh
```

# Pupper v3 LD06 / SLAM / Nav2 development

This fork includes the current Pupper v3 Raspberry Pi navigation work:

- LD06 LiDAR integration through `ldlidar_stl_ros2`, publishing `/scan` with the `base_laser` frame.
- `pupper_slam` launch/config files for SLAM Toolbox mapping on ROS 2 Jazzy.
- `pupper_odometry` nodes/config for IMU-assisted odometry and `odom -> base_link` support.
- `pupper_nav` launch/config files for Nav2, AMCL, map server, planner, controller, behavior tree navigator, and lifecycle bringup.
- Updated robot launch/config/URDF/RViz files so the LiDAR, odometry, SLAM, and navigation stack can share a standard TF/topic layout.
- `scripts_local/` deployment, diagnostics, and Pi operation helpers used during LD06, SLAM, EKF/odometry, map saving, and Nav2 verification.

The known-good Nav2 runbook is tracked in `NAV2_RUNBOOK.md`. The current quick start on the robot is:

```sh
~/pupperv3-monorepo/scripts_local/pi_start_nav.sh pupper_map_ekf_v1
```

The latest verified AMCL initial pose is configured in `ros2_ws/src/pupper_nav/config/nav2_params.yaml`:

```text
x = 7.20
y = 4.40
yaw = 2.60
```

Useful helper entry points:

```sh
~/pupperv3-monorepo/scripts_local/pi_start_slam.sh
~/pupperv3-monorepo/scripts_local/pi_save_map.sh
~/pupperv3-monorepo/scripts_local/pi_nav_status.sh
~/pupperv3-monorepo/scripts_local/pi_nav_initialpose.sh 7.20 4.40 2.60
~/pupperv3-monorepo/scripts_local/pi_nav_goal.sh 7.50 4.00 2.44
```

# Docs

Please see the [docs](https://pupper-v3-documentation.readthedocs.io/en/latest/)!

# Notes
* Camera FPS is 10hz by default. Adjustable in `ros2_ws/src/neural_controller/launch/config.yaml` with the `FrameDurationLimits: [100000, 100000]` parameter

# Development

## Adding animations

1. Hold L1 until BAG status icon turns green to indicate mcap bag recording in process
1. Move Pupper through desired motion
1. Press R1 to stop recording
1. View recorded mcap file in foxglove to verify animation
1. Move bag to pupperv3-monorepo/bags
1. On the robot use `scripts/mcap_to_csv.py [path_to_mcap] -s ABSOLUTE_START_TIME -e ABSOLUTE_END_TIME` to convert to csv
1. Copy csv to ros2_ws/src/animation_controller_py/launch/animations
1. Rebuild ros2 workspace with `./build.sh`
1. Update pupster.py with animation nickname
1. If editing animation frame rate or fade time, make sure to edit both config.yaml and pupster.yaml

## Camera
Launch mock camera and detection nodes so you can experiment with vision with simulated robot
```sh
ros2 launch hailo detection_with_mock_camera_launch.py
```

Launch Foxglove bridge so you can see detections in Foxglove studio
```sh
ros2 run foxglove_bridge foxglove_bridge
```
