#!/usr/bin/env bash

set -euo pipefail

#######################################
# CONFIG
#######################################
ROBOT_USER="ubuntu"

MAX_RETRIES=5
RETRY_DELAY=3

ROBOT_SETUP="/etc/turtlebot4/setup.bash"
ROBOT_ALIASES="/etc/turtlebot4/aliases.bash"

#######################################
# ARGUMENTS
#######################################
if [[ $# -ne 3 ]]; then
  echo "Usage: $0 <namespace> <robot_ip> <robot_password>"
  echo "Example: $0 robot_1 192.168.0.226 turtlebot4"
  exit 1
fi

NS="$1"
ROBOT_IP="$2"
ROBOT_PASS="$3"
NS_SLASH="/$NS"

#######################################
# HELPERS
#######################################
log() {
  echo -e "\n[$(date '+%H:%M:%S')] $1"
}

cleanup() {
  log "Stopping all processes..."
  pkill -P $$ || true
  exit 0
}

trap cleanup SIGINT SIGTERM

#######################################
# KILL EXISTING ROS2 NODES (SIGKILL)
#######################################
kill_existing_ros2_nodes() {
  sshpass -p "$ROBOT_PASS" ssh \
    -o StrictHostKeyChecking=no \
    -o UserKnownHostsFile=/dev/null \
    "$ROBOT_USER@$ROBOT_IP" <<'EOF'
source /etc/turtlebot4/setup.bash
source /etc/turtlebot4/aliases.bash

for pattern in \
  "sync_slam_toolbox" \
  "lifecycle_manager" \
  "bt_navigator" \
  "controller_server" \
  "planner_server" \
  "behavior_server" \
  "smoother_server" \
  "waypoint_follower" \
  "velocity_smoother"; do

  PIDS=$(ps -eo pid,cmd | grep -F "$pattern" | grep -v grep | grep -v "bash -lc" | grep -v "pgrep" | awk '{print $1}')

  if [[ -n "$PIDS" ]]; then
    echo "Killing $pattern: $PIDS"
    kill -9 $PIDS || true
  fi
done
EOF
}

#######################################
# START SLAM
#######################################
start_slam() {
  log "Starting SLAM (namespace=${NS})"

  sshpass -p "$ROBOT_PASS" ssh \
    -o StrictHostKeyChecking=no \
    -o UserKnownHostsFile=/dev/null \
    "$ROBOT_USER@$ROBOT_IP" \
    "bash -lc '
      source ${ROBOT_SETUP}
      source ${ROBOT_ALIASES}
      ros2 launch turtlebot4_navigation slam.launch.py namespace:=${NS}
    '" 2>&1 | tee slam.log
}

check_slam_success() {
  grep -q "Registering sensor" slam.log \
    && ! grep -Ei "\[[^]]*(warn|error)[^]]*\]" slam.log
}

#######################################
# START NAV2
#######################################
start_nav2() {
  log "Starting Nav2 (namespace=${NS})"

  sshpass -p "$ROBOT_PASS" ssh \
    -o StrictHostKeyChecking=no \
    -o UserKnownHostsFile=/dev/null \
    "$ROBOT_USER@$ROBOT_IP" \
    "bash -lc '
      source ${ROBOT_SETUP}
      source ${ROBOT_ALIASES}
      ros2 launch turtlebot4_navigation nav2.launch.py namespace:=${NS_SLASH}
    '" 2>&1 | tee nav2.log
}

check_nav2_success() {
  grep -Ei "StaticLayer:.*Resizing static layer" nav2.log \
    && ! grep -Ei "\[[^]]*(warn|error)[^]]*\]" nav2.log
}

#######################################
# SLAM RETRY LOOP
#######################################
for ((i=1; i<=MAX_RETRIES; i++)); do
  log "SLAM attempt $i/$MAX_RETRIES"
  rm -f slam.log

  kill_existing_ros2_nodes
  start_slam & SLAM_PID=$!

  sleep 10

  if check_slam_success; then
    log "SLAM started successfully"
    break
  else
    log "SLAM failed, retrying..."
    kill $SLAM_PID 2>/dev/null || true
    sleep "$RETRY_DELAY"
  fi

  if [[ $i -eq $MAX_RETRIES ]]; then
    log "SLAM failed after $MAX_RETRIES attempts"
    exit 1
  fi
done

#######################################
# NAV2 RETRY LOOP
#######################################
for ((i=1; i<=MAX_RETRIES; i++)); do
  log "Nav2 attempt $i/$MAX_RETRIES"
  rm -f nav2.log

  start_nav2 & NAV2_PID=$!

  sleep 30

  if check_nav2_success; then
    log "Nav2 started successfully"
    break
  else
    log "Nav2 failed, retrying..."
    kill $NAV2_PID 2>/dev/null || true
    sleep "$RETRY_DELAY"
  fi

  if [[ $i -eq $MAX_RETRIES ]]; then
    log "Nav2 failed after $MAX_RETRIES attempts"
    exit 1
  fi
done

#######################################
# START RVIZ (LOCAL)
#######################################
log "Starting RViz"
ros2 launch turtlebot4_viz view_robot.launch.py namespace:=${NS} &

#######################################
# START CORE MODULE
#######################################
log "Starting core module"
python3 -m core.main --namespace "${NS_SLASH}" &

#######################################
# START WEB MODULE
#######################################
log "Starting web backend"
python3 -m web.backend.main --namespace "${NS_SLASH}" &

log "✅ Robot stack fully started for namespace ${NS}"
wait
