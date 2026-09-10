#!/usr/bin/env bash
# Ubuntu 24.04: docs/07 Controller E2E as a 3x3 tmux grid inside one Terminator window.
#
# terminator's own saved-[layouts] loader (-g/-l) is unreliable on 2.1.3
# (upstream: "Layouts menu not working" gnome-terminator/terminator#718,
# "layout add/save does nothing" #881 -- confirmed against this repo's
# terminator too: -l fell back to a single default window instead of the
# 9-pane layout). tmux's split/tiled-layout is scripted instead; terminator
# just opens one window that attaches to it.
set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP=/opt/ros/jazzy/setup.bash
WORKSPACE_SETUP="$PROJECT_DIR/install/setup.bash"
SESSION=e2e

for required in terminator tmux docker; do
  command -v "$required" >/dev/null || { echo "Missing: $required" >&2; exit 1; }
done
[[ -f "$PROJECT_DIR/.env" ]] || { echo "Missing $PROJECT_DIR/.env" >&2; exit 1; }
[[ -f "$ROS_SETUP" && -f "$WORKSPACE_SETUP" ]] || {
  echo "Build first: colcon build --symlink-install" >&2; exit 1;
}
tmux has-session -t "$SESSION" 2>/dev/null && {
  echo "tmux session '$SESSION' already running -- 'tmux kill-session -t $SESSION' first." >&2
  exit 1
}

pane_command() {
  local command="$1"
  # No shell-quoting needed: tmux send-keys types this into the pane's own
  # interactive shell, same as if a person typed it. The shell prompt
  # returning after $command exits IS "kept open" -- no exec/trap tricks.
  printf 'cd %q && source %q && source %q && export ROS_DOMAIN_ID=%q && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && %s' \
    "$PROJECT_DIR" "$ROS_SETUP" "$WORKSPACE_SETUP" "${ROS_DOMAIN_ID:-20}" "$command"
}

# This pane starts vision only after its RealSense PointCloud2 topic exists.
REALSENSE_CMD='ros2 run realsense2_camera realsense2_camera_node --ros-args -r __ns:=/ -r __node:=camera -p enable_color:=true -p enable_depth:=true -p depth_module.depth_profile:=848x480x15 -p rgb_camera.color_profile:=1280x720x15 -p align_depth.enable:=true -p enable_rgbd:=true -p enable_sync:=true -p pointcloud.enable:=true -p pointcloud.stream_filter:=2 -p enable_accel:=false -p enable_gyro:=false -p initial_reset:=true & camera_pid=$!; trap "kill $camera_pid 2>/dev/null || true" EXIT; until ros2 topic list | grep -qx /camera/depth/color/points; do echo "Waiting for RealSense point cloud..."; sleep 1; done; docker compose up -d vision; wait "$camera_pid"'

TITLES=("MoveIt2 / M0609" "RealSense + vision" "DB node" "Position estimation" "Voice command" "Echo command result" "Echo component result" "Echo task status" "Controller")
CMDS=(
  'ros2 launch dsr_moveit_config_m0609 start.launch.py mode:=real name:=dsr01 model:=m0609 gripper:=rg2 host:=192.168.1.100 gui:=true'
  "$REALSENSE_CMD"
  'set -a; source .env; set +a; ros2 run kit_db db_node'
  'ros2 run kit_robot position_estimation'
  'ros2 run kit_voice get_command'
  'ros2 topic echo /kit/command_result'
  'ros2 topic echo /kit/component_result'
  'ros2 topic echo /kit/task_status'
  'ros2 run kit_robot controller --ros-args --params-file src/kit_robot/resource/controller.yaml -p restart_delay_sec:=60.0'
)

cd "$PROJECT_DIR"
docker compose up -d postgres mongodb
docker compose ps postgres mongodb

tmux new-session -d -s "$SESSION" -c "$PROJECT_DIR"
for _ in 1 2 3 4 5 6 7 8; do
  tmux split-window -t "$SESSION:0" -c "$PROJECT_DIR"
  tmux select-layout -t "$SESSION:0" tiled >/dev/null
done
tmux set-option -t "$SESSION" pane-border-status top
sleep 1  # newly split panes' shells need a moment before they'll accept send-keys
for i in "${!CMDS[@]}"; do
  tmux select-pane -t "$SESSION:0.$i" -T "${TITLES[$i]}"
  tmux send-keys -t "$SESSION:0.$i" "$(pane_command "${CMDS[$i]}")" C-m
done

terminator -x tmux attach -t "$SESSION" &

echo "Started PostgreSQL/MongoDB and opened a 9-pane tmux grid ('$SESSION') in Terminator."
echo "Reattach any time with: tmux attach -t $SESSION"
