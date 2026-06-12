#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
NODE_DIR="$ROOT/adapters/ros2/rne_ros2_node"

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

cd "$NODE_DIR"
./generate_cargo_config.sh

echo "Running convert unit tests..."
cargo test --manifest-path "$NODE_DIR/Cargo.toml"

echo "Building native ROS 2 node..."
cargo build --release --manifest-path "$NODE_DIR/Cargo.toml"

echo "Running bridge smoke test..."
"$NODE_DIR/target/release/rne_ros2_node"
