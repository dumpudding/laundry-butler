#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash
source /home/laundrybutler/camera_ws/install/setup.bash
source /home/laundrybutler/piper_ws/install/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
export LAUNDRY_BUTLER_DATA_ROOT="${LAUNDRY_BUTLER_DATA_ROOT:-/home/laundrybutler/Aloha Shared SSD/dxx_data/0727_300eps}"
export LAUNDRY_BUTLER_CAMERA_LAUNCH="${LAUNDRY_BUTLER_CAMERA_LAUNCH:-$ROOT_DIR/cameras/multi_camera_rgb.launch.py}"
export LAUNDRY_BUTLER_ARM_LAUNCH="${LAUNDRY_BUTLER_ARM_LAUNCH:-$ROOT_DIR/arms/dual_arm_observe.launch.py}"
export LAUNDRY_BUTLER_MIN_FREE_GB="${LAUNDRY_BUTLER_MIN_FREE_GB:-20}"

exec python3 "$ROOT_DIR/data_collection/data_collection_gui.py"
