#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BRIDGE_DIR="$ROOT/adapters/ros2/rne_ros2_bridge"
BRIDGE_PID=""

cleanup() {
  if [[ -n "$BRIDGE_PID" ]]; then
    kill "$BRIDGE_PID" 2>/dev/null || true
    wait "$BRIDGE_PID" 2>/dev/null || true
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

if ! python3 -c "import rclpy" 2>/dev/null; then
  echo "rclpy not found; install ros-\${ROS_DISTRO:-jazzy}-rclpy (e.g. sudo apt install ros-jazzy-rclpy)" >&2
  exit 1
fi

echo "Running convert unit tests..."
python3 test_ros_convert.py
python3 test_sim_control.py

echo "Running synthetic bridge smoke (no rne_py)..."
python3 run_node.py

PY_MINOR="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
VENV="$ROOT/.venv"

echo "Building rne_py into ${VENV}..."
python3 -m venv "$VENV"
"$VENV/bin/pip" install -q --upgrade pip maturin
"$VENV/bin/maturin" develop -m "$ROOT/crates/rne_py/Cargo.toml" --release
export PYTHONPATH="$VENV/lib/python${PY_MINOR}/site-packages:${PYTHONPATH:-}"

echo "Running live simulation bridge smoke..."
export RNE_ROS2_HOLD_SECS="${RNE_ROS2_HOLD_SECS:-60}"
python3 run_node.py &
BRIDGE_PID=$!

SERVICE_READY=0
for _ in $(seq 1 150); do
  if ros2 service list 2>/dev/null | grep -q '/get_simulation_state'; then
    SERVICE_READY=1
    break
  fi
  if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
    echo "rne_ros2_bridge exited before services became available" >&2
    wait "$BRIDGE_PID" || true
    exit 1
  fi
  sleep 0.1
done

if [[ "$SERVICE_READY" -ne 1 ]]; then
  echo "timed out waiting for /get_simulation_state (15s)" >&2
  exit 1
fi

sleep 0.2
for topic in /clock /points /tf; do
  echo "Checking ${topic}..."
  ros2 topic echo "$topic" --once
done

echo "Checking get_simulation_state service..."
timeout 20 ros2 service call /get_simulation_state simulation_interfaces/srv/GetSimulationState "{}"

echo "Python ROS 2 bridge smoke passed"
