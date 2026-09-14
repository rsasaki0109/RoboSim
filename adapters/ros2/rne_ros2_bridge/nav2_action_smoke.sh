#!/usr/bin/env bash
# Smoke test for the Nav2-compatible /navigate_to_pose action server.
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

cd "$BRIDGE_DIR"

echo "Starting nav bridge node..."
python3 nav_node.py &
NODE_PID=$!
sleep 2.0
if ! kill -0 "$NODE_PID" 2>/dev/null; then
  echo "nav_node.py exited early" >&2
  exit 1
fi

echo "Checking /navigate_to_pose action..."
actions="$(timeout 20 ros2 action list)"
[[ "$actions" == *"/navigate_to_pose"* ]]
for action in /compute_path_to_pose /follow_path /follow_waypoints /spin /backup; do
  echo "Checking ${action}..."
  [[ "$actions" == *"${action}"* ]]
done
echo "Checking /load_map service..."
services="$(timeout 20 ros2 service list)"
[[ "$services" == *"/load_map"* ]]

echo "Sending a NavigateToPose goal..."
goal_output="$(
  timeout 60 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
    --feedback 2>&1
)"
echo "$goal_output" | tail -5

echo "Checking odometry advanced to the goal..."
odom_x="$(timeout 20 ros2 topic echo /odom --field pose.pose.position.x --once | grep -E '^-?[0-9]' | head -1)"
echo "odom x = ${odom_x}"
if ! awk "BEGIN {exit !(${odom_x} > 0.8)}"; then
  echo "expected the base to reach the goal" >&2
  exit 1
fi

echo "Sending a ComputePathToPose goal..."
path_output="$(timeout 30 ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 2.0}, orientation: {w: 1.0}}}, use_start: false}")"
[[ "$path_output" == *"SUCCEEDED"* ]]

echo "Sending a Spin goal..."
spin_output="$(timeout 30 ros2 action send_goal /spin nav2_msgs/action/Spin \
  "{target_yaw: 0.5, time_allowance: {sec: 5}}")"
[[ "$spin_output" == *"SUCCEEDED"* ]]

echo "Sending a BackUp goal..."
backup_output="$(timeout 30 ros2 action send_goal /backup nav2_msgs/action/BackUp \
  "{target: {x: 0.2}, speed: 0.2, time_allowance: {sec: 5}}")"
[[ "$backup_output" == *"SUCCEEDED"* ]]

echo "Checking the ros2_control boundary..."
[[ "$(timeout 20 ros2 action list)" == *"/joint_trajectory_controller/follow_joint_trajectory"* ]]
[[ "$(timeout 20 ros2 topic list)" == *"/dynamic_joint_states"* ]]
jtraj_output="$(timeout 30 ros2 action send_goal \
  /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['left_wheel_joint', 'right_wheel_joint'], points: [{positions: [1.0, 1.0], velocities: [3.0, 3.0], time_from_start: {sec: 0}}, {positions: [2.0, 2.0], velocities: [3.0, 3.0], time_from_start: {sec: 0, nanosec: 300000000}}]}}")"
[[ "$jtraj_output" == *"SUCCEEDED"* ]]

echo "Nav2 NavigateToPose action smoke passed"
