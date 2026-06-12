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
python3 test_ros_convert.py

if [[ -d "$ROOT/.venv/lib/python3.12/site-packages" ]]; then
  export PYTHONPATH="$ROOT/.venv/lib/python3.12/site-packages:${PYTHONPATH:-}"
fi

python3 run_node.py
