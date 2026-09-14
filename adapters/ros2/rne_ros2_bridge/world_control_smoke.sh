#!/usr/bin/env bash
# Smoke test for the Gazebo-compatible entity services on run_node.py.
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

echo "Starting run_node with a hold window..."
RNE_ROS2_HOLD_SECS=10 python3 run_node.py &
NODE_PID=$!
sleep 3.0
if ! kill -0 "$NODE_PID" 2>/dev/null; then
  echo "run_node.py exited early" >&2
  exit 1
fi

echo "Listing entities..."
entities="$(timeout 20 ros2 service call /get_entities simulation_interfaces/srv/GetEntities '{}' 2>&1)"
echo "$entities" | tail -4
[[ "$entities" == *"base_link"* ]]

echo "Spawning an entity..."
spawned="$(timeout 20 ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity \
  "{name: 'test_box', allow_renaming: true, uri: 'box'}" 2>&1)"
echo "$spawned" | tail -4
[[ "$spawned" == *"test_box"* ]]

echo "Getting the entity state..."
state="$(timeout 20 ros2 service call /get_entity_state simulation_interfaces/srv/GetEntityState \
  "{entity: 'test_box'}" 2>&1)"
[[ "$state" == *"result=1"* ]]

echo "Deleting the entity..."
deleted="$(timeout 20 ros2 service call /delete_entity simulation_interfaces/srv/DeleteEntity \
  "{entity: 'test_box'}" 2>&1)"
[[ "$deleted" == *"result=1"* ]]

echo "Gazebo-compatible entity service smoke passed"
