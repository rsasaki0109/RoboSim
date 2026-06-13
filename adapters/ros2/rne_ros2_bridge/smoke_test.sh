#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BRIDGE_DIR="$ROOT/adapters/ros2/rne_ros2_bridge"

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

echo "Running convert unit tests..."
python3 test_ros_convert.py

echo "Running synthetic bridge smoke (no rne_py)..."
(
  unset PYTHONPATH
  python3 run_node.py
)

PY_MINOR="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
VENV="$ROOT/.venv"

echo "Building rne_py into ${VENV}..."
python3 -m venv "$VENV"
"$VENV/bin/pip" install -q --upgrade pip maturin
"$VENV/bin/maturin" develop -m "$ROOT/crates/rne_py/Cargo.toml" --release
export PYTHONPATH="$VENV/lib/python${PY_MINOR}/site-packages:${PYTHONPATH:-}"

echo "Running live simulation bridge smoke..."
python3 run_node.py &
BRIDGE_PID=$!

sleep 0.5
for topic in /clock /points /tf; do
  echo "Checking ${topic}..."
  ros2 topic echo "$topic" --once
done

wait "$BRIDGE_PID"

echo "Python ROS 2 bridge smoke passed"
