#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash
source /home/laundrybutler/camera_ws/install/setup.bash

set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
export LAUNDRY_BUTLER_CAPTURE_ROOT="${LAUNDRY_BUTLER_CAPTURE_ROOT:-$ROOT_DIR/cameras/output}"
export LAUNDRY_BUTLER_CAMERA_LAUNCH="${LAUNDRY_BUTLER_CAMERA_LAUNCH:-$ROOT_DIR/cameras/multi_camera_rgb.launch.py}"

exec python3 "$ROOT_DIR/cameras/camera_capture_gui.py"
