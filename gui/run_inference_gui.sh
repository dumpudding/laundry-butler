#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
OPENPI_ROOT="${LAUNDRY_BUTLER_OPENPI_ROOT:-/home/laundrybutler/Downloads/openpi-main}"
VENV="${LAUNDRY_BUTLER_INFERENCE_VENV:-$HOME/.cache/laundry-butler/inference-client}"

source /opt/ros/jazzy/setup.bash
source /home/laundrybutler/camera_ws/install/setup.bash
source /home/laundrybutler/piper_ws/install/setup.bash

set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"

if ! command -v uv >/dev/null 2>&1; then
    echo "uv is required. Expected ~/.local/bin/uv or another uv on PATH." >&2
    exit 2
fi

if [[ ! -d "$OPENPI_ROOT/packages/openpi-client" ]]; then
    echo "OpenPI client package not found: $OPENPI_ROOT/packages/openpi-client" >&2
    exit 3
fi

if [[ ! -x "$VENV/bin/python" ]]; then
    echo "Creating inference client environment: $VENV"
    uv venv --python /usr/bin/python3 --system-site-packages "$VENV"
    uv pip install --python "$VENV/bin/python" -e "$OPENPI_ROOT/packages/openpi-client"
fi

if ! "$VENV/bin/python" -c 'import openpi_client, rclpy, cv_bridge, PyQt5, piper_msgs' >/dev/null 2>&1; then
    echo "Refreshing inference client dependencies..."
    uv pip install --python "$VENV/bin/python" -e "$OPENPI_ROOT/packages/openpi-client"
fi

exec "$VENV/bin/python" "$ROOT_DIR/gui/inference_gui.py"
