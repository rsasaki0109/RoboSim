#!/usr/bin/env bash
# Live smoke test for the Nav2-facing RNE nav bridge (no simulation_interfaces).
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BRIDGE_DIR="$ROOT/adapters/ros2/rne_ros2_bridge"
NODE_PID=""

cleanup() {
  if [[ -n "$NODE_PID" ]]; then
    kill "$NODE_PID" 2>/dev/null || true
    wait "$NODE_PID" 2>/dev/null || true
  fi
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

if ! python3 -c "import rclpy" 2>/dev/null; then
  echo "rclpy not found" >&2
  exit 1
fi

cd "$BRIDGE_DIR"

echo "Running nav mapping round-trip..."
python3 test_nav_roundtrip.py

echo "Starting nav bridge node..."
python3 nav_node.py &
NODE_PID=$!

sleep 2.0
if ! kill -0 "$NODE_PID" 2>/dev/null; then
  echo "nav_node.py exited early" >&2
  exit 1
fi

for topic in /clock /odom /tf /scan /map /plan /joint_states; do
  echo "Checking ${topic}..."
  timeout 20 ros2 topic echo "$topic" --once >/dev/null
done

echo "Checking /cmd_vel subscription..."
ros2 topic info /cmd_vel | grep -q "Subscription count: 1"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}"

sleep 0.5
echo "Checking odometry advanced after cmd_vel..."
odom_x="$(timeout 20 ros2 topic echo /odom --field pose.pose.position.x --once | grep -E '^-?[0-9]' | head -1)"
echo "odom x = ${odom_x}"
if ! awk "BEGIN {exit !(${odom_x} > 0.0)}"; then
  echo "expected forward motion from /cmd_vel" >&2
  exit 1
fi

echo "Nav2-facing bridge smoke passed"
