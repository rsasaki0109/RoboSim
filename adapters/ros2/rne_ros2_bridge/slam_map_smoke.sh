#!/usr/bin/env bash
# Verifies that nav_node.py can publish an RNE SLAM map as a ROS map.
#
# 1. Runs the physics+LiDAR SLAM example to write a PGM/YAML map.
# 2. Starts nav_node.py with RNE_MAP_FILE pointing at it.
# 3. Checks the published /map metadata matches the SLAM grid.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BRIDGE_DIR="$ROOT/adapters/ros2/rne_ros2_bridge"
MAP_DIR="${RNE_SLAM_MAP_DIR:-$(mktemp -d)}"
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

echo "Generating SLAM map in ${MAP_DIR}..."
RNE_SLAM_MAP_DIR="$MAP_DIR" cargo run --manifest-path "$ROOT/Cargo.toml" -q -p nav_slam_physics \
  --example 98_nav_slam_physics >/dev/null

if [[ ! -f "$MAP_DIR/map.yaml" ]]; then
  echo "expected $MAP_DIR/map.yaml" >&2
  exit 1
fi

cd "$BRIDGE_DIR"
echo "Starting nav bridge with the SLAM map..."
RNE_MAP_FILE="$MAP_DIR/map.yaml" python3 nav_node.py &
NODE_PID=$!
sleep 3

if ! kill -0 "$NODE_PID" 2>/dev/null; then
  echo "nav_node.py exited early" >&2
  exit 1
fi

width="$(timeout 20 ros2 topic echo /map --field info.width --once | grep -E '^[0-9]' | head -1)"
height="$(timeout 20 ros2 topic echo /map --field info.height --once | grep -E '^[0-9]' | head -1)"
echo "published SLAM map = ${width}x${height}"
if [[ "$width" != "240" || "$height" != "160" ]]; then
  echo "unexpected SLAM map size ${width}x${height}" >&2
  exit 1
fi

echo "SLAM map -> Nav2 bridge smoke passed"
