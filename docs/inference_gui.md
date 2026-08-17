# Laundry Butler inference GUI

## Canonical layout

Inference implementation:

```text
inference/
  inference_gui.py
  inference_policy_server.py
  piper_openpi_config.py
```

Launcher:

```text
gui/
  run_inference_gui.sh
```

Runtime/local-only state:

```text
checkpoints/
logs/inference/
inference/.venv-ros-client/
```

`gui/` contains the launcher. The inference implementation itself belongs under
`inference/`.

The inference GUI owns the policy-server subprocess and starts/stops it as
part of the GUI workflow; no separate server launcher is required.

## Start

```bash
cd ~/laundry-butler
./gui/run_inference_gui.sh
```

The inference launcher defaults to:

```bash
ROS_DOMAIN_ID=0
```

Override it explicitly when required:

```bash
ROS_DOMAIN_ID=0 ./gui/run_inference_gui.sh
```

## Machine-specific path overrides

Tracked files should not need editing just because a workstation path changes.

Supported launcher overrides:

```text
LAUNDRY_BUTLER_OPENPI_ROOT
LAUNDRY_BUTLER_CAMERA_WS
LAUNDRY_BUTLER_PIPER_WS
LAUNDRY_BUTLER_INFERENCE_VENV
UV_BIN
```

Current defaults:

```text
OpenPI:        /home/laundrybutler/Downloads/openpi-main
camera_ws:     /home/laundrybutler/camera_ws
piper_ws:      /home/laundrybutler/piper_ws
client venv:   inference/.venv-ros-client
```

## Current working runtime behavior

The August 17 working inference behavior is the baseline. This repository
cleanup intentionally does not alter the physical rollout/control logic.

Current behavior includes:

- GUI-managed policy-server lifecycle;
- checkpoint selection;
- three camera previews;
- joint/camera health monitoring;
- dry inference;
- guarded physical rollout;
- latest-trajectory Return Home;
- reduced GUI preview load;
- no burst catch-up after publish-loop delays;
- per-command arm-feedback gating;
- current feedback-grace behavior from `inference/inference_gui.py`.

Changes to timing, stale-feedback thresholds, execution horizon, or command
gating should be treated as control changes and tested separately from
repository/path cleanup.

## Local-only files

Do not commit:

```text
checkpoints/
inference/.venv-ros-client/
logs/inference/
*.pre_*
```

Do not keep full `.pre_*` copies of tracked Python files in the repository.
Git history is the source backup.
