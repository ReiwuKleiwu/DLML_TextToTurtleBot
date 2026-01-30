#!/bin/bash

#######################################
# CONFIG
#######################################

SIM_LOG="/tmp/turtlebot_sim.log"

#######################################
# HELPERS
#######################################

log() {
  echo -e "\n[$(date '+%H:%M:%S')] $1"
}

cleanup() {
    log "\nShutting down all processes..."
    # Kill the background process group
    pkill -P $$
    exit
}
trap cleanup SIGINT SIGTERM

log "Starting TurtleBot 4 Simulation..."

#######################################
# START SIMULATOR
#######################################

ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py \
    slam:=true nav2:=true rviz:=true 2>&1 | tee "$SIM_LOG" &

SIM_PID=$!

#######################################
# WAIT FOR SIM TO BOOT UP
#######################################
log "Waiting for Nav2 stack to initialize..."
# Check for string in stdout to tell if initialization completed
while ! grep -q "Creating bond timer" "$SIM_LOG"; do
    # Check if sim crashed while waiting
    if ! kill -0 $SIM_PID 2>/dev/null; then
        log "Error: Simulator failed to start."
        exit 1
    fi
    sleep 1
done

log "\n======================================================\nNav2 is READY!\n======================================================"

#######################################
# START CORE MODULE
#######################################
log "Starting core module"
python3 -m core.main --namespace "" --use_turtlebot_sim &

#######################################
# START WEB MODULE
#######################################
log "Starting web backend"
python3 -m web.backend.main --namespace "" &

log "✅ Robot stack fully started"
wait