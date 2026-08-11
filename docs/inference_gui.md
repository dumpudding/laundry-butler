# Laundry Butler inference GUI

Files to add to `~/laundry-butler`:

```text
inference/
  inference_gui.py
  policy_server.py
  piper_openpi_config.py
gui/
  run_inference_gui.sh
  run_inference_server.sh
```

## Design

The GUI follows the existing data-collection GUI structure: PyQt5, saved window geometry, reduced-rate camera previews, joint rates/liveness, managed subsystem launch controls, and a ROS `MultiThreadedExecutor` with reentrant callback groups.

The policy server remains a separate process because it runs inside the OpenPI/JAX Python 3.11 environment. The GUI runs in a small Python 3.12 ROS-compatible client venv with `--system-site-packages` and only `openpi-client` added.

## Start

Terminal 1:

```bash
cd ~/laundry-butler
newgrp vglusers
./gui/run_inference_server.sh
```

Terminal 2:

```bash
cd ~/laundry-butler
./gui/run_inference_gui.sh
```

The GUI can start/configure CAN, cameras, and observation-arm nodes itself. If those nodes are already running, it attaches to them and refuses duplicate launches.

## GUI functions

- 3 live camera previews with Hz and message age.
- Left/right joint values, Hz, and age.
- `can_left` / `can_right` state, bitrate, RX/TX counters.
- Policy server TCP health and metadata query.
- Command-path status; no command publishers exist while idle/dry-inference.
- One-shot dry inference with no robot motion.
- Full physical rollout.
- Default 50 actions per replan at 30 Hz.
- Sequential joint slew cap of 0.08 rad/step.
- Gripper slew cap of 0.015 m/step and `[0, 0.08]` command clamp.
- Hard raw policy jump abort (default 0.35 rad) for gross mismatch.
- Joint-limit, finite-value, observation-age, arm-age, command-subscriber, and server checks.
- Live replan/inference timing and raw-vs-published step metrics.
- 14-D current/raw/published first-target comparison table.
- `STOP / HOLD` button.
- Automatic JSONL logs containing each replan's full raw and published action chunks.
- Debug snapshot saving: all 3 camera PNGs + state/rates/ages JSON.
- Clean ROS shutdown guard to avoid calling `rcl_shutdown` twice after an earlier shutdown.

## Default checkpoint

`policy_server.py` defaults to:

```text
~/laundry-butler/checkpoints/piper_pi05_full_v1/15000
```

Override without editing code:

```bash
LAUNDRY_BUTLER_CHECKPOINT=/path/to/checkpoint ./gui/run_inference_server.sh
```

Logs go under `inference/logs/`; snapshots under `inference/snapshots/`.
