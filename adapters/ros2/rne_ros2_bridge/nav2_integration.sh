#!/usr/bin/env bash
# End-to-end integration: drive the RNE nav bridge with a real Nav2 stack.
#
# Starts nav_node.py (RNE publishes /map, /scan, /odom, /tf, /clock and
# subscribes /cmd_vel), launches Nav2 navigation, sends a NavigateToPose goal,
# and verifies that Nav2's controller moves the RNE base.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BRIDGE_DIR="$ROOT/adapters/ros2/rne_ros2_bridge"
NODE_PID=""
NAV2_PID=""

cleanup() {
  for pid in "$NAV2_PID" "$NODE_PID"; do
    if [[ -n "$pid" ]]; then
      kill "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "ROS 2 setup.bash not found under /opt/ros" >&2
  exit 1
fi

PARAMS_FILE="${NAV2_PARAMS_FILE:-/opt/ros/${ROS_DISTRO}/share/nav2_bringup/params/nav2_params.yaml}"
if [[ ! -f "$PARAMS_FILE" ]]; then
  echo "Nav2 params not found at $PARAMS_FILE; install ros-${ROS_DISTRO}-nav2-bringup" >&2
  exit 1
fi

cd "$BRIDGE_DIR"

echo "Starting RNE nav bridge..."
python3 nav_node.py &
NODE_PID=$!
sleep 2.0

echo "Launching Nav2 navigation..."
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  autostart:=true \
  params_file:="$PARAMS_FILE" &
NAV2_PID=$!

echo "Waiting for /navigate_to_pose action server..."
ACTION_READY=0
for _ in $(seq 1 120); do
  if ros2 action list 2>/dev/null | grep -q '/navigate_to_pose'; then
    ACTION_READY=1
    break
  fi
  if ! kill -0 "$NAV2_PID" 2>/dev/null; then
    echo "Nav2 exited before the action server appeared" >&2
    exit 1
  fi
  sleep 0.5
done
if [[ "$ACTION_READY" -ne 1 ]]; then
  echo "timed out waiting for /navigate_to_pose (60s)" >&2
  exit 1
fi

# Wait for the planner/controller action servers and costmaps too; the BT
# otherwise races the lifecycle activation and aborts on a follow_path timeout.
for _ in $(seq 1 60); do
  if ros2 action list 2>/dev/null | grep -q '/follow_path' \
    && ros2 action list 2>/dev/null | grep -q '/compute_path_to_pose'; then
    break
  fi
  sleep 0.5
done

echo "Waiting for Nav2 costmaps to activate..."
for _ in $(seq 1 60); do
  if ros2 topic list 2>/dev/null | grep -q '/global_costmap/costmap'; then
    break
  fi
  sleep 0.5
done
sleep 5

echo "Sending NavigateToPose goal to (2.0, 0.0)..."
timeout 90 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  || true

odom_x="$(timeout 20 ros2 topic echo /odom --field pose.pose.position.x --once | grep -E '^-?[0-9]' | head -1)"
echo "final odom x = ${odom_x}"
if ! awk "BEGIN {exit !(${odom_x} > 0.3)}"; then
  echo "Nav2 did not drive the RNE base forward (odom x=${odom_x})" >&2
  exit 1
fi

echo "Nav2 <-> RNE integration passed"
