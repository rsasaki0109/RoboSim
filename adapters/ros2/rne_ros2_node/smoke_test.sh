#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
NODE_DIR="$ROOT/adapters/ros2/rne_ros2_node"
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
  export ROS_PREFIX=/opt/ros/jazzy
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  export ROS_PREFIX=/opt/ros/humble
else
  echo "ROS 2 setup.bash not found under /opt/ros" >&2
  exit 1
fi

cd "$NODE_DIR"
./generate_cargo_config.sh

echo "Running convert unit tests..."
cargo test --manifest-path "$NODE_DIR/Cargo.toml"

echo "Building native ROS 2 node..."
cargo build --release --manifest-path "$NODE_DIR/Cargo.toml"

echo "Running bridge smoke test..."
unset RNE_ROS2_MODE
export RNE_ROS2_HOLD_SECS="${RNE_ROS2_HOLD_SECS:-60}"
pkill -f "target/release/rne_ros2_node" 2>/dev/null || true
sleep 0.5
SMOKE_LOG="$(mktemp "${TMPDIR:-/tmp}/rne_ros2_smoke.XXXXXX")"
"$NODE_DIR/target/release/rne_ros2_node" >"$SMOKE_LOG" 2>&1 &
NODE_PID=$!

wait_for_log_line() {
  local pattern="$1"
  local timeout_tenths="$2"
  for _ in $(seq 1 "$timeout_tenths"); do
    if grep -q "$pattern" "$SMOKE_LOG" 2>/dev/null; then
      return 0
    fi
    if ! kill -0 "$NODE_PID" 2>/dev/null; then
      echo "rne_ros2_node exited before log matched: $pattern" >&2
      tail -20 "$SMOKE_LOG" >&2 || true
      return 1
    fi
    sleep 0.1
  done
  echo "timed out waiting for log line: $pattern" >&2
  tail -20 "$SMOKE_LOG" >&2 || true
  return 1
}

wait_for_joint_state() {
  local needle="$1"
  local timeout_secs="$2"
  local out=""
  for _ in $(seq 1 "$((timeout_secs * 2))"); do
    out="$(timeout 2 ros2 topic echo /joint_states --once 2>/dev/null || true)"
    if echo "$out" | grep -q "$needle"; then
      return 0
    fi
    if ! kill -0 "$NODE_PID" 2>/dev/null; then
      break
    fi
    sleep 0.5
  done
  echo "expected /joint_states to include ${needle}, last message:${out:+ (no match)}" >&2
  return 1
}

SERVICE_READY=0
for _ in $(seq 1 150); do
  if ros2 service list 2>/dev/null | grep -q '/get_simulation_state'; then
    SERVICE_READY=1
    break
  fi
  if ! kill -0 "$NODE_PID" 2>/dev/null; then
    echo "rne_ros2_node exited before services became available" >&2
    wait "$NODE_PID" || true
    exit 1
  fi
  sleep 0.1
done

if [[ "$SERVICE_READY" -ne 1 ]]; then
  echo "timed out waiting for /get_simulation_state (15s)" >&2
  exit 1
fi

wait_for_log_line "final base_x=" 600 || exit 1
wait_for_log_line "holding ROS graph" 600 || exit 1
tail -5 "$SMOKE_LOG" || true

echo "Checking get_simulation_state service..."
timeout 20 ros2 service call /get_simulation_state simulation_interfaces/srv/GetSimulationState "{}"

echo "Checking /points LiDAR width..."
POINTS_WIDTH=$(
  timeout 20 ros2 topic echo /points --once --field width --no-lost-messages 2>/dev/null \
    | grep -E '^[0-9]+$' \
    | tail -1 \
    || true
)
if [[ -z "$POINTS_WIDTH" || "$POINTS_WIDTH" -lt 8 ]]; then
  echo "expected /points width >= 8, got '${POINTS_WIDTH}'" >&2
  exit 1
fi

echo "Checking /scan publication..."
if ! timeout 20 ros2 topic echo /scan --once --field angle_increment --no-lost-messages >/dev/null 2>&1; then
  echo "failed to receive /scan" >&2
  exit 1
fi

echo "Checking /joint_states publication..."
wait_for_joint_state "left_wheel_joint" 30 || exit 1

echo "Running mobile manipulator bridge smoke..."
kill "$NODE_PID" 2>/dev/null || true
wait "$NODE_PID" 2>/dev/null || true
NODE_PID=""
rm -f "$SMOKE_LOG"
SMOKE_LOG="$(mktemp "${TMPDIR:-/tmp}/rne_ros2_smoke.XXXXXX")"

export RNE_ROS2_MODE=mobile_manipulator
export RNE_ROS2_HOLD_SECS=30
"$NODE_DIR/target/release/rne_ros2_node" >"$SMOKE_LOG" 2>&1 &
NODE_PID=$!

for _ in $(seq 1 150); do
  if ros2 service list 2>/dev/null | grep -q '/get_simulation_state'; then
    break
  fi
  if ! kill -0 "$NODE_PID" 2>/dev/null; then
    echo "mobile rne_ros2_node exited before services became available" >&2
    wait "$NODE_PID" || true
    exit 1
  fi
  sleep 0.1
done

wait_for_log_line "final base_x=" 600 || exit 1
wait_for_log_line "holding ROS graph" 600 || exit 1
tail -5 "$SMOKE_LOG" || true

echo "Checking /cmd_vel subscription exists..."
CMD_VEL_SUBS=$(
  ros2 topic info /cmd_vel 2>/dev/null | awk '/Subscription count/ {print $3}' || true
)
if [[ -z "$CMD_VEL_SUBS" || "$CMD_VEL_SUBS" -lt 1 ]]; then
  echo "expected /cmd_vel subscription on mobile bridge, got count=${CMD_VEL_SUBS:-0}" >&2
  exit 1
fi

echo "Checking mobile /joint_states publication..."
wait_for_joint_state "shoulder_joint" 30 || exit 1

MM_JOINT_WIDTH=$(
  timeout 20 ros2 topic echo /joint_states --once 2>/dev/null \
    | grep -c '_joint' \
    || true
)
if [[ "$MM_JOINT_WIDTH" -lt 4 ]]; then
  echo "expected mobile /joint_states to include 4 joints, got count=${MM_JOINT_WIDTH}" >&2
  exit 1
fi

echo "Checking /gripper_command subscription exists..."
GRIPPER_SUBS=$(
  ros2 topic info /gripper_command 2>/dev/null | awk '/Subscription count/ {print $3}' || true
)
if [[ -z "$GRIPPER_SUBS" || "$GRIPPER_SUBS" -lt 1 ]]; then
  echo "expected /gripper_command subscription on mobile bridge, got count=${GRIPPER_SUBS:-0}" >&2
  exit 1
fi

echo "Checking ee_link TF frame is published..."
EE_TF_FOUND=0
for _ in $(seq 1 10); do
  if timeout 10 ros2 topic echo /tf --once --no-lost-messages 2>/dev/null | grep -q "ee_link"; then
    EE_TF_FOUND=1
    break
  fi
done
if [[ "$EE_TF_FOUND" -ne 1 ]]; then
  echo "expected /tf to include the ee_link frame in mobile manipulator mode" >&2
  exit 1
fi

echo "Checking /arm_joint_position subscription exists..."
ARM_POS_SUBS=$(
  ros2 topic info /arm_joint_position 2>/dev/null | awk '/Subscription count/ {print $3}' || true
)
if [[ -z "$ARM_POS_SUBS" || "$ARM_POS_SUBS" -lt 1 ]]; then
  echo "expected /arm_joint_position subscription on mobile bridge, got count=${ARM_POS_SUBS:-0}" >&2
  exit 1
fi

echo "Checking /arm_joint_trajectory subscription exists..."
ARM_TRAJ_SUBS=$(
  ros2 topic info /arm_joint_trajectory 2>/dev/null | awk '/Subscription count/ {print $3}' || true
)
if [[ -z "$ARM_TRAJ_SUBS" || "$ARM_TRAJ_SUBS" -lt 1 ]]; then
  echo "expected /arm_joint_trajectory subscription on mobile bridge, got count=${ARM_TRAJ_SUBS:-0}" >&2
  exit 1
fi

rm -f "$SMOKE_LOG"
echo "ROS 2 smoke tests passed."
