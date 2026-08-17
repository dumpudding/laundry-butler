#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INFERENCE_DIR="$ROOT_DIR/inference"

OPENPI_ROOT="${LAUNDRY_BUTLER_OPENPI_ROOT:-/home/laundrybutler/Downloads/openpi-main}"
CAMERA_WS="${LAUNDRY_BUTLER_CAMERA_WS:-/home/laundrybutler/camera_ws}"
PIPER_WS="${LAUNDRY_BUTLER_PIPER_WS:-/home/laundrybutler/piper_ws}"
VENV="${LAUNDRY_BUTLER_INFERENCE_VENV:-$INFERENCE_DIR/.venv-ros-client}"

require_file() {
    if [[ ! -f "$1" ]]; then
        echo "Required file not found: $1" >&2
        exit 2
    fi
}

require_file /opt/ros/jazzy/setup.bash
require_file "$CAMERA_WS/install/setup.bash"
require_file "$PIPER_WS/install/setup.bash"

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1090
source "$CAMERA_WS/install/setup.bash"
# shellcheck disable=SC1090
source "$PIPER_WS/install/setup.bash"

set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export LAUNDRY_BUTLER_OPENPI_ROOT="$OPENPI_ROOT"

UV_BIN="${UV_BIN:-$(command -v uv || true)}"
if [[ -z "$UV_BIN" && -x "$HOME/.local/bin/uv" ]]; then
    UV_BIN="$HOME/.local/bin/uv"
fi
if [[ -z "$UV_BIN" ]]; then
    echo "uv is required. Install it or put it on PATH." >&2
    exit 3
fi

OPENPI_CLIENT="$OPENPI_ROOT/packages/openpi-client"
if [[ ! -d "$OPENPI_CLIENT" ]]; then
    echo "OpenPI client package not found: $OPENPI_CLIENT" >&2
    exit 4
fi

if [[ ! -x "$VENV/bin/python" ]]; then
    echo "Creating inference client environment: $VENV"
    "$UV_BIN" venv --python /usr/bin/python3 --system-site-packages "$VENV"
    "$UV_BIN" pip install --python "$VENV/bin/python" -e "$OPENPI_CLIENT"
fi

if ! "$VENV/bin/python" -c \
    'import openpi_client, rclpy, cv_bridge, PyQt5, piper_msgs' \
    >/dev/null 2>&1; then
    echo "Refreshing inference client dependencies..."
    "$UV_BIN" pip install --python "$VENV/bin/python" -e "$OPENPI_CLIENT"
fi

cd "$ROOT_DIR"
exec "$VENV/bin/python" "$INFERENCE_DIR/inference_gui.py"
