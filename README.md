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
- No commanded arm movement, replay, or inference has been performed under the rebuilt setup.
- The next workstream is a unified data-collection interface with episode recording, browsing, playback, validation, and human annotations.

## Repository layout

```text
gui/
├── run_camera_capture_gui.sh
└── run_arm_status_gui.sh

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

docs/
└── for_llms.txt
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

Both interfaces default to:

```text
ROS_DOMAIN_ID=88
```

The camera and arm GUIs currently record their subsystems separately. The upcoming data-collection interface will own synchronized full-episode recording.

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

## Data-collection roadmap

The unified data-collection interface should treat one episode as:

```text
Immutable MCAP
+ episode metadata
+ annotation sidecar
+ validation report
```

Initial functions:

- Start and stop camera and observation-only arm nodes.
- Run preflight checks before recording.
- Record all required camera and arm topics into one episode.
- Assign an episode ID, task, garment, operator, initial-state level, and notes.
- Browse recorded episodes without modifying their MCAP files.
- Play synchronized front, left, and right camera streams.
- Display arm state alongside playback.
- Mark success, failure reason, demonstration quality, and excluded ranges.
- Create and edit timestamped natural-language task stages.
- Validate topic presence, duration, message counts, rates, and timestamp coverage.

Recommended episode layout:

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

MCAP files remain immutable. Human and future automatic annotations are stored in editable, versioned sidecar files.

## Repository boundaries

Add to Git:

- Application source
- ROS launch wrappers
- Human-facing GUI launchers
- Documentation
- Schemas and manifests
- Small evaluation metadata

Do not add to Git:

- `/home/laundrybutler/piper_ws`
- `/home/laundrybutler/camera_ws`
- `cameras/output/`
- `arms/output/`
- Raw MCAP recordings
- Snapshots, videos, datasets, or converted training data
- ROS workspace `build/`, `install/`, or `log/`
- Model weights or checkpoints
- Host-specific CAN, udev, or systemd configuration