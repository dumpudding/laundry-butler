# laundry-butler

WORK IN PROGRESS

Laundry folding task trained on Piper using VLA and reinforcement-learning
methods.
Made in collaboration with OpenAI's ChatGPT.

## Current status

- Three Orbbec Dabai DC1 RGB cameras are mapped, launched, viewed, and recorded.
- All three cameras have passed individual and simultaneous 30 fps validation.
- The Piper vendor workspace builds successfully under ROS 2 Jazzy.
- Stable left/right CAN identities and persistent interface names are configured.
- Both arms have passed isolated, observation-only feedback validation.
- No commanded arm motion has been performed under the rebuilt setup.
- The next workstream is an observation-first arm interface and combined
  camera-and-arm recording.

## Piper vendor workspace

The Piper ROS 2 driver is maintained separately from this Git repository:

```text
/home/laundrybutler/piper_ws
```

It was cloned directly from the original AgileX factory computer and pinned to:

```text
e38e0c62319140116ab176a9d1d3c4b51aa6401e
```

The workspace builds successfully under ROS 2 Jazzy and Python 3.12:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/laundrybutler/piper_ws

colcon build --symlink-install \
    --packages-select piper_msgs piper_description piper
```

The new workstation uses Piper SDK 0.6.1. The original factory computer used
SDK 0.5.0. All SDK methods used by the factory Piper node are available with
compatible signatures on the new workstation.

Only `piper_single_ctrl` is verified functional. The registered
`piper_ms_ctrl` and `piper_read_master` entry points reference absent Python
modules and must not be used.

## Verified Piper CAN identity

The physical workstation USB-A ports are labeled `LARM` and `RARM`. Preserve
these connections and do not move the cables between USB ports.

| Logical interface | Stable adapter serial | Current USB path | Physical role |
|---|---|---|---|
| `can_left` | `0029001C4148570D20343133` | `3-7:1.0` | LARM |
| `can_right` | `003200184148570A20343133` | `3-11.2:1.0` | RARM |

Persistent names are provided by host-specific systemd link files:

```text
/etc/systemd/network/10-piper-can-left.link
/etc/systemd/network/11-piper-can-right.link
```

These files are machine-specific and are not tracked in this repository.

Both arm CAN buses use:

```text
bitrate: 1000000
txqueuelen: 1000
```

Do not run the copied factory `can_config.sh`; its USB paths belong to the
original AgileX computer.

## Piper validation and safety status

Completed:

- Both CAN interfaces reached `UP`, `LOWER_UP`, and `ERROR-ACTIVE`.
- Passive CAN monitoring showed active traffic with no new errors.
- Both Piper nodes ran simultaneously with `auto_enable=false`.
- Position, joint-command, and enable endpoints were isolated.
- `/puppet/joint_left` and `/puppet/joint_right` published near 200 Hz.
- Both arms reported `arm_status=0` and `err_code=0`.
- Each node sent only the expected 13 Piper initialization query frames.
- No enable, gripper, joint-motion, Cartesian-motion, or replay commands were
  sent.

Not completed:

- Teleoperation under the rebuilt setup.
- Commanded arm movement.
- Replay or inference.
- A tracked project-owned observation launch wrapper.
- Combined camera-and-arm MCAP validation.
- Final training-topic schema validation.

The manufacturer manual remains the source of truth for physical setup and
safety.

Before power-on:

- place both arms in the required horizontal/reset pose;
- manually close both grippers;
- keep the emergency stop accessible;
- preserve all labeled USB connections.

Before replay or inference:

- physically disconnect both master-arm power plugs;
- verify command topic, subscriber, message type, units, joint order, gripper
  representation, limits, rate, CAN identity, start pose, and emergency stop.

`auto_enable=false` prevents the startup enable loop, but does not make the
factory node inherently receive-only. Command topics, enable topics, and enable
services must still be isolated for observation-only operation.

<!-- CAMERA-HARDWARE-MAPPING:START -->
## Camera hardware mapping

The three Orbbec Dabai DC1 cameras were identified visually using their stable
serial identities.

| Role | ROS namespace | Stable serial | Stable colour-device link |
|---|---|---|---|
| Front/top camera | `camera_f` | `CC1WC52009R` | `/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._Dabai_DC1_CC1WC52009R-video-index0` |
| Left wrist camera | `camera_l` | `CC1WC52006V` | `/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._Dabai_DC1_CC1WC52006V-video-index0` |
| Right wrist camera | `camera_r` | `CC1WC52012P` | `/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._Dabai_DC1_CC1WC52012P-video-index0` |

Do not persist numeric `/dev/videoN` names. They may change after reconnecting
devices or rebooting.

The left and right wrist cameras share part of the USB topology. Camera roles
must be selected by serial number rather than inferred from parent hubs or
enumeration order.
<!-- CAMERA-HARDWARE-MAPPING:END -->

## Camera runtime

The factory Orbbec ROS 2 source is built separately under:

```text
/home/laundrybutler/camera_ws
```

The tracked project launch file is:

```text
cameras/multi_camera_rgb.launch.py
```

It starts:

```text
camera_f -> CC1WC52009R
camera_l -> CC1WC52006V
camera_r -> CC1WC52012P
```

Current camera settings:

```text
640x480
30 fps
MJPG
RGB only
depth disabled
IR disabled
point clouds disabled
```

All three cameras passed simultaneous validation. A 9.26-second combined MCAP
contained approximately 30 RGB frames per second from each camera, with only
four transport-layer losses across 1,651 total messages.

For camera-health checks, use lightweight `camera_info` cadence and MCAP message
counts. Do not use `ros2 topic hz` on the large raw-image topic as the primary
measurement because the Python subscriber can under-report the real stream.

Depth is not currently used as an input to the existing OpenPI π0.5 training
path.

## Camera capture interface

Launch the interface:

```bash
cd /home/laundrybutler/laundry-butler
./capture/run_camera_capture_gui.sh
```

The interface supports:

- starting and stopping the tracked RGB camera launch;
- left, front, and right previews;
- separate stream and preview frame-rate displays;
- selected-camera MCAP recording;
- snapshots;
- output organization in clock-aligned 30-minute folders.

Generated output is written under:

```text
capture/output/
```

This directory is ignored by Git.

`Stream FPS` reflects the lightweight camera publication cadence.
`View FPS` reflects the intentionally reduced GUI preview rate. MCAP recording
continues at the full camera publication rate.

## Repository boundaries

Add source code, launch files, documentation, schemas, configuration templates,
and small evaluation metadata to Git.

Do not add:

- `/home/laundrybutler/piper_ws`;
- `/home/laundrybutler/camera_ws`;
- `capture/output/`;
- ROS workspace build, install, or log directories;
- MCAP recordings;
- snapshots, videos, or datasets;
- model weights or checkpoints;
- system-specific CAN or device configuration.
