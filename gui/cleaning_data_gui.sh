#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash
if [[ -f /home/laundrybutler/camera_ws/install/setup.bash ]]; then
  source /home/laundrybutler/camera_ws/install/setup.bash
fi
if [[ -f /home/laundrybutler/piper_ws/install/setup.bash ]]; then
  source /home/laundrybutler/piper_ws/install/setup.bash
fi
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
export LAUNDRY_BUTLER_DATA_ROOT="${LAUNDRY_BUTLER_DATA_ROOT:-/home/laundrybutler/Aloha Shared SSD/dxx_data}"
export LAUNDRY_BUTLER_MIN_EPISODE_SECONDS="${LAUNDRY_BUTLER_MIN_EPISODE_SECONDS:-20}"

exec python3 "$ROOT_DIR/data_collection/data_cleansing_gui.py"
