# Laundry Butler

Laundry Butler is a bimanual shirt-folding system using two AgileX Piper
arms, three RGB cameras, ROS 2 Jazzy, and OpenPI pi0.5.

The current target is reliable folding of one known shirt from a mostly
consistent spread starting configuration.

## Current status

Implemented:

- synchronized three-camera and dual-arm MCAP recording;
- live camera, joint, end-pose, arm-status, rate, and liveness views;
- automatic readiness checks before recording;
- per-episode metadata and validation reports;
- isolated synchronized playback;
- episode review, disposition, outcome, and notes;
- safe GUI deletion by moving episodes to `.trash`.

Current training direction:

- official OpenPI pi0.5;
- full fine-tuning;
- no LoRA;
- no frozen components;
- absolute 14D joint data in LeRobot;
- OpenPI relative actions for 12 arm joints;
- absolute left and right grippers;
- no Cartesian-relative actions;
- 30 Hz action grid;
- 50 predicted actions;
- initially execute 20 actions before replanning;
- global batch size 32 on eight A800 GPUs.

See [`docs/for_llms.txt`](docs/for_llms.txt) for the authoritative data,
training, inference, and failure-point decisions.

## Run the data-collection GUI

```bash
cd /home/laundrybutler/laundry-butler
./gui/run_data_collection_gui.sh
```

The launcher sources ROS 2 Jazzy, the camera workspace, and the Piper
workspace. The default domain is:

```text
ROS_DOMAIN_ID=88
```

Required packages:

```bash
sudo apt install python3-pyqt5 ros-jazzy-rosbag2-storage-mcap
```

## Recording workflow

1. Put both arms in the required reset pose.
2. Close both grippers manually.
3. Keep the labeled left/right USB and CAN connections unchanged.
4. Keep the emergency stop accessible.
5. Start or attach to the cameras and observation-only arm nodes.
6. Confirm all camera and arm streams are live.
7. Use initial state `level_1_spread`.
8. Use the instruction `Fold the shirt.`
9. Start the episode; readiness is checked automatically.
10. Stop and mark the episode usable or unusable.
11. Review the episode through isolated playback.

Before recording more demonstrations, adjust or lock wrist-camera
exposure and verify both left and right arm topics.

## Recorded data

Each episode contains one MCAP and two metadata sidecars:

```text
episode_<timestamp>_<task>/
├── bag/
│   ├── *.mcap
│   └── metadata.yaml
├── episode.json
├── validation.json
└── recorder.log
```

Recorded topics:

```text
/camera_f/color/image_raw
/camera_f/color/camera_info
/camera_l/color/image_raw
/camera_l/color/camera_info
/camera_r/color/image_raw
/camera_r/color/camera_info
/puppet/joint_left
/puppet/joint_right
/puppet/end_pose_left
/puppet/end_pose_right
/piper_left_ctrl_node/arm_status
/piper_right_ctrl_node/arm_status
```

`/master/joint_left` and `/master/joint_right` are intentionally excluded.

The current external dataset root is:

```text
/home/laundrybutler/Aloha Shared SSD/dxx_data/0727_300eps
```

Raw MCAPs and training data are not stored in Git.

## Joint format

The physical state and action representation is 14D:

```text
[left joint0..5, left gripper,
 right joint0..5, right gripper]
```

Arm joints are radians. Grippers are measured openings in metres.

LeRobot stores absolute state and action values. OpenPI converts only the
12 arm joints to actions relative to the current state and converts them
back to absolute targets during inference.

Do not use the previous Cartesian-relative pipeline.

## Readiness and validation

Readiness checks:

- ROS 2, CAN utilities, and MCAP support;
- expected camera and arm nodes;
- all required recording topics;
- receipt of fresh camera and arm samples;
- `can_left` and `can_right` state and bitrate;
- command-topic isolation;
- available storage.

A topic being listed in rosbag metadata does not prove that messages were
recorded. Validation must check nonzero message counts and rates.

## Playback isolation

Playback remaps recorded topics under:

```text
/laundry_butler/viewer/<original-topic>
```

Playback is observation-only and does not publish to live robot command
topics.

## Not implemented

- timeline seeking;
- stage annotation;
- excluded time ranges;
- automatic recovery from a stream dropping during recording;
- dataset conversion through the GUI;
- robot replay;
- general-purpose homing, reset, or robot-enable controls outside the guarded inference workflow.

## Repository policy

Do not commit:

- raw MCAP files;
- converted datasets;
- checkpoints;
- training outputs;
- caches;
- ROS `build/`, `install/`, or `log/` directories;
- local workspaces;
- secrets or machine credentials.

## Inference GUI

The current inference implementation lives under `inference/` and is launched
from `gui/run_inference_gui.sh`.

```bash
cd ~/laundry-butler
./gui/run_inference_gui.sh
```

The inference GUI manages the policy-server subprocess directly; no separate
server launcher is required. The inference launcher defaults to
`ROS_DOMAIN_ID=0`.

See `docs/inference_gui.md` for the current layout and runtime notes.
