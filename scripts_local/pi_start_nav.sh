#!/usr/bin/env bash
set -eo pipefail

MAP_NAME="${1:-pupper_map_ekf_v1}"
MAP_PATH="${MAP_NAME}"

if [[ "${MAP_PATH}" != /* ]]; then
  MAP_PATH="/home/pi/maps/${MAP_NAME}.yaml"
fi

source /opt/ros/jazzy/setup.bash
if [[ -f "$HOME/ldlidar_ros2_ws/install/setup.bash" ]]; then
  source "$HOME/ldlidar_ros2_ws/install/setup.bash"
fi
if [[ -f "$HOME/nav2_ws/install/setup.bash" ]]; then
  source "$HOME/nav2_ws/install/setup.bash"
fi
source "$HOME/pupperv3-monorepo/ros2_ws/install/setup.bash"

export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

echo "[pupper_nav] map: ${MAP_PATH}"
test -f "${MAP_PATH}"
test -f "${MAP_PATH%.yaml}.pgm" || true

if ! timeout 8 ros2 topic echo /scan --once >/dev/null 2>&1; then
  echo "[pupper_nav] /scan not active; starting LD06 LiDAR."
  pkill -f ldlidar_stl_ros2_node 2>/dev/null || true
  pkill -f "ld06.launch.py" 2>/dev/null || true
  nohup ros2 launch ldlidar_stl_ros2 ld06.launch.py \
    > /tmp/pupper_ld06.log 2>&1 &
  for _ in $(seq 1 20); do
    if timeout 8 ros2 topic echo /scan --once >/dev/null 2>&1; then
      echo "[pupper_nav] /scan is active."
      break
    fi
    sleep 1
  done
fi

if ! timeout 8 ros2 topic echo /scan --once >/dev/null 2>&1; then
  echo "[pupper_nav] ERROR: /scan did not become active. See /tmp/pupper_ld06.log." >&2
  exit 1
fi

if ! timeout 8 ros2 topic echo /odom --once >/dev/null 2>&1; then
  echo "[pupper_nav] ERROR: /odom did not become active. Is robot.service healthy?" >&2
  exit 1
fi

auto_activate_nav2() {
  local nodes=(
    map_server
    amcl
    controller_server
    smoother_server
    planner_server
    behavior_server
    bt_navigator
  )

  echo "[pupper_nav] lifecycle watchdog: waiting for Nav2 nodes."
  sleep 20

  for attempt in $(seq 1 6); do
    echo "[pupper_nav] lifecycle watchdog: attempt ${attempt}."
    local all_active=1

    for node in "${nodes[@]}"; do
      local state
      state="$(timeout 8 ros2 lifecycle get "/${node}" 2>&1 || true)"
      echo "[pupper_nav] lifecycle watchdog: ${node}: ${state}"

      if [[ "${state}" == *"active [3]"* ]]; then
        continue
      fi

      all_active=0

      if [[ "${state}" == *"unconfigured [1]"* ]]; then
        timeout 20 ros2 lifecycle set "/${node}" configure || true
        sleep 1
      fi

      if [[ "${state}" == *"inactive [2]"* || "${state}" == *"unconfigured [1]"* ]]; then
        timeout 25 ros2 lifecycle set "/${node}" activate || true
        sleep 2
      fi
    done

    if [[ "${all_active}" -eq 1 ]]; then
      echo "[pupper_nav] lifecycle watchdog: all Nav2 nodes are active."
      return 0
    fi

    sleep 5
  done

  echo "[pupper_nav] lifecycle watchdog: Nav2 nodes are not all active; manual check needed." >&2
  return 1
}

pkill -f "ros2 launch pupper_nav nav.launch.py" 2>/dev/null || true
pkill -f "nav2_.*_server|bt_navigator|amcl|map_server|lifecycle_manager_navigation" 2>/dev/null || true
sleep 1

auto_activate_nav2 > /tmp/pupper_nav_lifecycle_watchdog.log 2>&1 &

ros2 launch pupper_nav nav.launch.py map:="${MAP_PATH}"
