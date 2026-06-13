#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
NODE_DIR="$ROOT/adapters/ros2/rne_ros2_node"

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
"$NODE_DIR/target/release/rne_ros2_node" &
NODE_PID=$!

SERVICE_READY=0
for _ in $(seq 1 150); do
  if ros2 service list 2>/dev/null | grep -q '/get_simulation_state'; then
    SERVICE_READY=1
    break
  fi
  sleep 0.1
done

if [[ "$SERVICE_READY" -ne 1 ]]; then
  echo "timed out waiting for /get_simulation_state (15s)" >&2
  kill "$NODE_PID" 2>/dev/null || true
  wait "$NODE_PID" 2>/dev/null || true
  exit 1
fi

echo "Checking get_simulation_state service..."
timeout 20 ros2 service call /get_simulation_state simulation_interfaces/srv/GetSimulationState "{}"

wait "$NODE_PID"
