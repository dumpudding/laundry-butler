#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
OPENPI_ROOT="${LAUNDRY_BUTLER_OPENPI_ROOT:-/home/laundrybutler/Downloads/openpi-main}"
CHECKPOINT="${LAUNDRY_BUTLER_CHECKPOINT:-/home/laundrybutler/laundry-butler/checkpoints/piper_pi05_full_v1/15000}"
PROMPT="${LAUNDRY_BUTLER_PROMPT:-Fold the shirt.}"
HOST="${LAUNDRY_BUTLER_POLICY_HOST:-127.0.0.1}"
PORT="${LAUNDRY_BUTLER_POLICY_PORT:-8000}"

if ! command -v uv >/dev/null 2>&1; then
    echo "uv is required." >&2
    exit 2
fi

if [[ ! -d "$OPENPI_ROOT/.venv" ]]; then
    echo "OpenPI environment missing. Run 'uv sync' in $OPENPI_ROOT first." >&2
    exit 3
fi

if ! nvidia-smi >/dev/null 2>&1; then
    echo "The current shell cannot access the NVIDIA GPU." >&2
    echo "For this workstation, run: newgrp vglusers" >&2
    echo "Then rerun this launcher." >&2
    exit 4
fi

cd "$OPENPI_ROOT"
exec uv run "$ROOT_DIR/gui/inference_policy_server.py" \
    --checkpoint "$CHECKPOINT" \
    --prompt "$PROMPT" \
    --host "$HOST" \
    --port "$PORT"
