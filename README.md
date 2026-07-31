# laundry-butler

**Work in progress**

Laundry-folding research on a bimanual Piper platform using vision-language-action and reinforcement-learning methods.

## Current status

- Three Orbbec Dabai DC1 RGB cameras are mapped by serial number and validated individually and simultaneously near 30 FPS.
- The camera GUI can launch the RGB camera nodes, show three previews, record selected camera topics to MCAP, and save snapshots.
- Stable `can_left` and `can_right` identities are configured at 1 Mbit/s.
- The dual-arm observation launch runs both Piper nodes with `auto_enable=false` and isolates all position, joint, and enable endpoints.
- The arm GUI shows CAN health, command-isolation status, joint and gripper feedback, end poses, arm status, and MCAP controls.
- A 3.125-second arm MCAP captured all six selected arm topics at approximately 200 Hz.
- The Stage 2 data-collection GUI can launch or attach to cameras and arms, run automatic readiness checks, record synchronized episodes, write metadata and validation sidecars, browse and review episodes, play recordings under an isolated viewer namespace, and move episodes reversibly to `.trash`.
- A July 31 USB-CAN failure was traced to a transient `gs_usb` USB transport state (`-71` / `EPROTO`). Physically reconnecting the affected adapter restored normal CAN traffic.
- No commanded arm movement, replay-to-robot, or inference has been performed under the rebuilt setup.
- Current work is runtime signal-drop diagnosis, recording-watchdog behavior, timeline review, and export.

## Repository layout

```text
gui/
├── run_camera_capture_gui.sh
├── run_arm_status_gui.sh
└── run_data_collection_gui.sh

cameras/
├── camera_capture_gui.py
├── multi_camera_rgb.launch.py
├── README.md
└── output/                       # Generated; ignored by Git

arms/
├── arm_status_gui.py
├── dual_arm_observe.launch.py
├── README.md
└── output/                       # Generated; ignored by Git

data_collection/
├── data_collection_gui.py
├── episode_store.py
├── preflight.py
├── validation.py
├── stage_vocabulary.json
├── schemas/
└── README.md

docs/
├── for_llms.txt
├── can_topic_restart.txt
└── data_collection_ui_plan.md

tests/
└── test_episode_store.py
```

## Start the interfaces

### Camera interface

```bash
cd /home/laundrybutler/laundry-butler
./gui/run_camera_capture_gui.sh
```

### Arm observation interface

```bash
cd /home/laundrybutler/laundry-butler
./gui/run_arm_status_gui.sh
```

### Unified data-collection interface

```bash
cd /home/laundrybutler/laundry-butler
./gui/run_data_collection_gui.sh
```

All three interfaces default to:

```text
ROS_DOMAIN_ID=88
```

The camera and arm GUIs remain useful for subsystem diagnostics and isolated recordings. Use the unified data-collection interface for synchronized full-episode recording and review.

## Camera subsystem

The local Orbbec vendor workspace is outside this repository:

```text
/home/laundrybutler/camera_ws
```

Tracked launch file:

```text
cameras/multi_camera_rgb.launch.py
```

| Role | ROS namespace | Serial |
|---|---|---|
| Front/top | `camera_f` | `CC1WC52009R` |
| Left wrist | `camera_l` | `CC1WC52006V` |
| Right wrist | `camera_r` | `CC1WC52012P` |

Current stream configuration:

```text
640 × 480
30 FPS
MJPG
RGB only
```

Camera roles are selected by serial number. Do not persist `/dev/videoN` device names.

See [`cameras/README.md`](cameras/README.md) for camera GUI usage and validation details.

## Arm subsystem

The local Piper vendor workspace is outside this repository:

```text
/home/laundrybutler/piper_ws
```

Pinned factory-source commit:

```text
e38e0c62319140116ab176a9d1d3c4b51aa6401e
```

Tracked observation launch:

```text
arms/dual_arm_observe.launch.py
```

| Interface | Adapter serial | Physical role |
|---|---|---|
| `can_left` | `0029001C4148570D20343133` | LARM |
| `can_right` | `003200184148570A20343133` | RARM |

The arm interface records these topics by default:

```text
/puppet/joint_left
/puppet/joint_right
/puppet/end_pose_left
/puppet/end_pose_right
/piper_left_ctrl_node/arm_status
/piper_right_ctrl_node/arm_status
```

`/puppet/joint_*` contains six arm joints followed by gripper feedback:

```text
position[0:6]   Six arm joints in radians
position[6]     Gripper feedback in metres
effort[6]       Scaled gripper effort feedback
```

`/master/joint_*` represents command-state feedback and is not used as measured physical state.

The arm GUI intentionally contains no enable, reset, stop, joint-command, Cartesian-command, gripper-command, replay, or inference controls.

See [`arms/README.md`](arms/README.md) for usage and safety constraints.

### USB-CAN recovery

A CAN interface may remain visible while its `gs_usb` adapter is stuck at the USB transport layer.

Recognized symptoms:

```text
RTNETLINK answers: Protocol error
Error -71 while reading timestamp
Couldn't start device (err=-71)
failed to xmit URB ... -ENOENT
```

When this occurs:

1. Stop `dual_arm_observe.launch.py` and both `piper_single_ctrl` processes.
2. Inspect recent kernel messages with `journalctl -k`.
3. If CAN reconfiguration still returns `Protocol error`, unplug the affected USB-CAN adapter for 5–10 seconds.
4. Reconnect it to the same labeled USB port.
5. Reconfigure it at 1 Mbit/s.
6. Confirm `UP`, `ERROR-ACTIVE`, and live frames with `candump` before restarting ROS.

Do not repeatedly relaunch ROS while raw CAN traffic is absent. Do not use `restart-ms`; these adapters do not support automatic bus-off restart.

## Safety

The manufacturer workflow and pinned factory source are the sources of truth.

Before powering or observing the arms:

- Place both arms in the required horizontal/reset pose.
- Manually close both grippers.
- Preserve the labeled USB and CAN connections.
- Keep the emergency stop accessible.

`auto_enable=false` prevents automatic startup enable, but it does not remove the factory command and enable endpoints. The project launch remaps those endpoints, and the GUI verifies their isolation before recording.

Before replay or inference:

- Physically disconnect or otherwise isolate both master-arm power plugs.
- Verify command topics and subscribers.
- Verify units, joint order, gripper representation, limits, command rate, CAN identity, and start pose.
- Keep the emergency stop accessible.

## Unified data collection

Run:

```bash
cd /home/laundrybutler/laundry-butler
./gui/run_data_collection_gui.sh
```

The Stage 2 interface currently provides:

- start or attach behavior for the three camera nodes and two observation-only Piper nodes;
- duplicate-launch refusal;
- automatic readiness checks at startup, after subsystem launch, and before recording when results are stale;
- camera previews and live left/right arm state;
- synchronized MCAP recording;
- immutable source MCAPs with editable metadata and validation sidecars;
- outcome values: not assessed, success, partial, and failure;
- dispositions: usable, needs review, and unusable;
- editable review notes;
- playback isolated under `/laundry_butler/viewer/*`;
- pause, resume, stop, and playback-rate controls;
- reversible deletion by moving complete episode directories to `.trash`.

The operator field was removed because it added friction without resolving a current ambiguity.

Episode output is controlled by `LAUNDRY_BUTLER_DATA_ROOT`. Large episode data must remain outside Git.

Episode layout:

```text
episode_<id>/
├── bag/
│   ├── *.mcap
│   └── metadata.yaml
├── episode.json
├── annotations.json
├── validation.json
└── recorder.log
```

MCAP files remain immutable. Human and future automatic annotations belong in editable, versioned sidecars.

Remaining work:

- diagnose camera or arm streams that disappear during active recording;
- add a watchdog that records dropout intervals and prevents automatic `usable` disposition;
- add timeline seeking, synchronized cursors, dropout markers, stage boundaries, and excluded ranges;
- export filtered manifests without modifying source episodes or including `.trash`.

## Repository boundaries

Add to Git:

- Application source
- ROS launch wrappers
- Human-facing GUI launchers
- Documentation
- Schemas and manifests
- Small evaluation metadata

- Tests
Do not add to Git:

- `/home/laundrybutler/piper_ws`
- `/home/laundrybutler/camera_ws`
- `cameras/output/`
- `arms/output/`
- `data_collection/output/`
- Raw MCAP recordings
- Snapshots, videos, datasets, or converted training data
- ROS workspace `build/`, `install/`, or `log/`
- Model weights or checkpoints
- Host-specific CAN, udev, or systemd configuration
