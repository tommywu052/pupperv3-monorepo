# Pupper v3 Nav2 Runbook

This guide records the working Nav2 bringup flow after the first successful navigation test.

## Current Working Pose

The current known-good AMCL initial pose is:

```text
x = 7.20
y = 4.40
yaw = 2.60
```

This is already configured in:

```text
~/pupperv3-monorepo/ros2_ws/install/pupper_nav/share/pupper_nav/config/nav2_params.yaml
```

and in the local source:

```text
C:\Nvidia\pupperv3\pupperv3-monorepo\ros2_ws\src\pupper_nav\config\nav2_params.yaml
```

If the robot is placed at the same starting point and heading, AMCL should initialize automatically when Nav2 starts.

## After Reboot

1. SSH into the Pi:

```bash
ssh pi@192.168.31.70
```

2. Confirm the robot service is running:

```bash
systemctl is-active robot.service
```

Expected:

```text
active
```

3. Start Nav2, LiDAR, and map bringup:

```bash
~/pupperv3-monorepo/scripts_local/pi_start_nav.sh pupper_map_ekf_v1
```

This script should:

- source ROS 2 Jazzy
- source the LiDAR workspace
- source the Nav2 source workspace
- source the Pupper workspace
- start LD06 LiDAR if `/scan` is not already active
- verify `/scan`
- verify `/odom`
- launch `pupper_nav`
- automatically run a lifecycle watchdog to recover inactive Nav2 nodes

## Quick Start Checklist

Run:

```bash
~/pupperv3-monorepo/scripts_local/pi_start_nav.sh pupper_map_ekf_v1
```

Then check:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ldlidar_ros2_ws/install/setup.bash
source ~/nav2_ws/install/setup.bash
source ~/pupperv3-monorepo/ros2_ws/install/setup.bash

for n in map_server amcl controller_server smoother_server planner_server behavior_server bt_navigator; do
  echo -n "$n: "
  ros2 lifecycle get /$n
done
```

All nodes should show:

```text
active [3]
```

If any node is not active, check the watchdog log:

```bash
cat /tmp/pupper_nav_lifecycle_watchdog.log
```

If the watchdog did not recover it, clear and restart:

```bash
~/pupperv3-monorepo/scripts_local/pi_stop_nav.sh
~/pupperv3-monorepo/scripts_local/pi_start_nav.sh pupper_map_ekf_v1
```

## Foxglove

Connect Foxglove to:

```text
ws://192.168.31.70:8765
```

If Foxglove does not connect or Nav2 custom topics look broken, restart the bridge with the Nav2 environment sourced:

```bash
pkill -TERM -f foxglove_bridge || true

source /opt/ros/jazzy/setup.bash
source ~/nav2_ws/install/setup.bash
source ~/pupperv3-monorepo/ros2_ws/install/setup.bash

nohup ros2 run foxglove_bridge foxglove_bridge \
  --ros-args -p port:=8765 -p address:=0.0.0.0 \
  > /tmp/foxglove_bridge_manual.log 2>&1 &
```

Check the WebSocket port:

```bash
ss -ltnp | grep 8765
```

## Status Checks

Source the environments first:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ldlidar_ros2_ws/install/setup.bash
source ~/nav2_ws/install/setup.bash
source ~/pupperv3-monorepo/ros2_ws/install/setup.bash
```

Check key topics:

```bash
ros2 topic list | grep -E '^/(scan|map|odom|tf|tf_static|amcl_pose|nav_cmd_vel|plan)$'
```

Expected topics include:

```text
/scan
/map
/odom
/tf
/tf_static
/amcl_pose
/nav_cmd_vel
/plan
```

Check Nav2 lifecycle:

```bash
for n in map_server amcl controller_server smoother_server planner_server behavior_server bt_navigator; do
  echo -n "$n: "
  ros2 lifecycle get /$n
done
```

Expected:

```text
active [3]
```

Check AMCL pose:

```bash
ros2 topic echo /amcl_pose --once
```

Check TF:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

## If Nav2 Starts Inactive

`pi_start_nav.sh` now starts a background lifecycle watchdog. It waits for Nav2 nodes, checks their lifecycle states, and tries to configure or activate nodes that are stuck in `unconfigured` or `inactive`.

Watchdog log:

```bash
cat /tmp/pupper_nav_lifecycle_watchdog.log
```

If you still want to manually activate nodes:

```bash
source /opt/ros/jazzy/setup.bash
source ~/nav2_ws/install/setup.bash
source ~/pupperv3-monorepo/ros2_ws/install/setup.bash

for n in map_server amcl controller_server smoother_server planner_server behavior_server bt_navigator; do
  ros2 lifecycle set /$n activate
  sleep 2
done
```

Then re-check lifecycle states.

## Clear And Restart Nav2

Create the stop script once if it does not exist:

```bash
cat > ~/pupperv3-monorepo/scripts_local/pi_stop_nav.sh <<'EOF'
#!/usr/bin/env bash
set -e

echo "[pupper_nav] stopping Nav2..."
pkill -TERM -f "ros2 launch pupper_nav nav.launch.py" || true
pkill -TERM -f "nav2_map_server|nav2_amcl|nav2_controller|nav2_planner|nav2_smoother|nav2_behaviors|nav2_bt_navigator|nav2_lifecycle_manager|map_server|amcl|bt_navigator" || true

sleep 3

pkill -KILL -f "ros2 launch pupper_nav nav.launch.py" || true
pkill -KILL -f "nav2_map_server|nav2_amcl|nav2_controller|nav2_planner|nav2_smoother|nav2_behaviors|nav2_bt_navigator|nav2_lifecycle_manager|map_server|amcl|bt_navigator" || true

echo "[pupper_nav] stopping optional LiDAR and Foxglove bridge..."
pkill -TERM -f "ldlidar_stl_ros2|ld06.launch.py|foxglove_bridge" || true

echo "[pupper_nav] remaining related processes:"
ps -eo pid,etime,args | grep -E "pupper_nav|nav2_|map_server|amcl|bt_navigator|ldlidar|foxglove_bridge" | grep -v grep || true

echo "[pupper_nav] done."
EOF

chmod +x ~/pupperv3-monorepo/scripts_local/pi_stop_nav.sh
```

Use it:

```bash
~/pupperv3-monorepo/scripts_local/pi_stop_nav.sh
~/pupperv3-monorepo/scripts_local/pi_start_nav.sh pupper_map_ekf_v1
```

## Setting Initial Pose Manually

If the robot is not placed at the saved starting pose, set AMCL manually.

Use the helper script:

```bash
~/pupperv3-monorepo/scripts_local/pi_nav_initialpose.sh X Y YAW
```

Known-good pose:

```bash
~/pupperv3-monorepo/scripts_local/pi_nav_initialpose.sh 7.20 4.40 2.60
```

Alternative service method:

```bash
ros2 service call /set_initial_pose nav2_msgs/srv/SetInitialPose "{pose: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, pose: {pose: {position: {x: 7.20, y: 4.40, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.963558185, w: 0.267498829}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]}}}"
```

In Foxglove, verify that:

- `/scan` overlays the map walls reasonably well
- the robot is not inside occupied black cells
- the pose is stable before sending goals

## First Goal Test

Use very short nearby goals only.

Known successful test:

```bash
~/pupperv3-monorepo/scripts_local/pi_nav_goal.sh 7.50 4.00 2.44
```

This completed with:

```text
Goal finished with status: SUCCEEDED
error_code: 0
```

For future tests, start with goals within 0.2-0.5 m.

Before sending a goal:

- keep joystick in hand
- ensure the path is clear
- confirm `/scan` still aligns with the map
- confirm Nav2 lifecycle nodes are active

## Stop Everything For Battery Change

Stop Nav2, LiDAR, Foxglove bridge, and robot service:

```bash
pkill -TERM -f "pupper_nav nav.launch.py" || true
pkill -TERM -f "nav2_map_server|nav2_amcl|nav2_controller|nav2_planner|nav2_smoother|nav2_behaviors|nav2_bt_navigator|nav2_lifecycle_manager" || true
pkill -TERM -f "ldlidar_stl_ros2" || true
pkill -TERM -f "foxglove_bridge" || true
sudo systemctl stop robot.service
```

Check remaining processes:

```bash
ps -eo pid,etime,args | grep -E "pupper_nav|map_server|amcl|controller_server|planner_server|bt_navigator|ldlidar|foxglove_bridge|neural_controller" | grep -v grep
```

## Notes

- `pupper_map_ekf_v1.yaml` `origin` is the map image origin, not the robot initial pose.
- The current default AMCL pose is stored in `pupper_nav/config/nav2_params.yaml`.
- If Foxglove shows only the grid, check WebSocket first, then reset the 3D panel view and ensure `/map`, `/scan`, `/tf`, `/tf_static`, and `/robot_description` are visible.
- The planner warning about inflation radius appeared during testing. It did not prevent the first goal from succeeding, but it should be tuned later.
- `number_of_recoveries` was nonzero in the first successful test, so controller/costmap tuning is the next development stage.
