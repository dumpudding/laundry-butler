#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash
source /home/laundrybutler/piper_ws/install/setup.bash

set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
export LAUNDRY_BUTLER_ARM_ROOT="${LAUNDRY_BUTLER_ARM_ROOT:-$ROOT_DIR/arms/output}"
export LAUNDRY_BUTLER_ARM_LAUNCH="${LAUNDRY_BUTLER_ARM_LAUNCH:-$ROOT_DIR/arms/dual_arm_observe.launch.py}"

exec python3 "$ROOT_DIR/arms/arm_status_gui.py"
