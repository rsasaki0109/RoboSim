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
export RNE_ROS2_HOLD_SECS="${RNE_ROS2_HOLD_SECS:-60}"
"$NODE_DIR/target/release/rne_ros2_node" &
NODE_PID=$!

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
