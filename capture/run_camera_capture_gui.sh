#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash
source /home/laundrybutler/camera_ws/install/setup.bash

set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
export LAUNDRY_BUTLER_CAPTURE_ROOT="${LAUNDRY_BUTLER_CAPTURE_ROOT:-$ROOT_DIR/capture/output}"

exec python3 "$ROOT_DIR/capture/camera_capture_gui.py"
