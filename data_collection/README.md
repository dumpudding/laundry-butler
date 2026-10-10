# Unified data-collection GUI — Stage 2

This interface records, reviews, and replays synchronized Laundry Butler demonstrations.

## Implemented

- responsive split-pane layout that scales from 1100×700 upward;
- saved window and splitter geometry;
- episode list using half the review pane by default, with a wider draggable divider and scrollable review/playback controls;
- Roboto Mono Bold request with a monospace fallback when the font is unavailable;
- three reduced-rate camera previews;
- live left/right arm observation for joints, end pose, status, rate, and liveness;
- independent demonstration target capture directly from the two CAN buses, with source-frame provenance;
- arm panels prioritize measured joints and demonstration targets, with scrolling for pose/status details in smaller windows;
- start blocked without real targets on both arms; command loss stops and preserves the episode as unusable;
- observation-only arm and camera launch controls;
- automatic read-only readiness checks at startup, after subsystem launch, and immediately before recording when stale;
- one MCAP directory per episode plus metadata and validation sidecars;
- editable episode outcome, disposition, and notes;
- synchronized episode playback remapped under `/laundry_butler/viewer/*` so playback does not publish onto live topics;
- playback pause/resume, stop, and 0.5×/1×/2× rates;
- safe episode deletion by moving the episode into `data_collection/output/.trash/` rather than permanently erasing it;
- post-recording topic-rate and timestamp-overlap validation.

Not implemented yet:

- timeline scrubbing or seeking;
- stage annotation and excluded time ranges;
- dataset conversion or export;
- automatic handling of a camera or arm stream that drops midway through an active recording;
- motion, replay-to-robot, inference, homing, reset, or enable controls.

## Run

```bash
cd /home/laundrybutler/laundry-butler
./gui/data_collection_gui.sh
```

The launcher sources ROS 2 Jazzy, the camera workspace, and the Piper workspace. It defaults to `ROS_DOMAIN_ID=88`.

Required packages:

```bash
sudo apt install python3-pyqt5 ros-jazzy-rosbag2-storage-mcap
```

The application requests the `Roboto Mono` font. When it is not installed, it falls back to another bold monospace font rather than failing.

## Readiness checks

The former manual “preflight” is now presented as **readiness**. It checks:

- ROS 2, CAN utilities, and the MCAP plugin;
- all expected camera and arm nodes;
- all 16 required recording topics (12 observations, 2 target streams, 2 provenance streams);
- receipt of fresh camera-info, joint, and arm-status samples;
- fresh, complete CAN joint-target cycles and an observed gripper target on each arm;
- `can_left` and `can_right` are UP, ERROR-ACTIVE, and at 1 Mbit/s;
- observation-only command topics and enable services remain isolated;
- enough free storage exists at the configured output path.

The checks are still necessary because a rosbag process can start successfully while recording missing topics or writing to an unsuitable disk. The user no longer needs to press the readiness button before every episode: the GUI rechecks automatically when required.

The GUI checks the most recent camera-info, joint, and arm-status samples received by its existing live subscriptions. All seven topics must have a sample no more than 3 seconds old. Playback samples do not count. This avoids creating temporary `ros2 topic echo` subscriptions that can time out during discovery while the live previews continue updating. Freshness is checked again immediately before recording, even when the other readiness checks are still valid. Standalone checks retain the CLI subscription fallback.

The command check uses the GUI's own receive-only SocketCAN capture, rather than ROS topic presence or the driver's cached `/master/joint_*` values. Both arms must send complete joint-target cycles with no gap longer than 0.5 seconds. Each cycle's three joint-pair frames must arrive within 0.1 seconds. A real gripper target is required initially; its last commanded opening and effort are held until the next gripper command. A CAN reconnect, command gap, or gripper-zeroing frame clears this context. Missing values are never filled from measured feedback or zeros. Only one collection GUI may publish the capture topics in the ROS domain.

This supports Piper V2 **joint-target teleoperation**. A controller sending only Cartesian, MIT, or feedback frames will remain blocked, with a message to activate a compatible target stream. The GUI monitors targets; it does not enable or move the arms.

## Workflow

1. Put both arms in the manufacturer-required horizontal/reset pose.
2. Manually close both grippers.
3. Preserve the labeled LARM and RARM USB/CAN connections.
4. Keep the emergency stop accessible.
5. Start or attach to cameras and observation-only arms.
6. Activate leader/handheld joint-target teleoperation, send initial gripper targets on both arms, and confirm both demonstration-target panels as well as the observations.
7. Fill in the episode metadata.
8. Select **Start episode**. The GUI runs readiness automatically when the previous result is stale.
9. Stop and keep, or stop and mark unusable.
10. Select the episode to edit outcome, disposition, and notes.
11. Play the episode to review synchronized camera and arm topics.

Under the current collection policy, **Stop and keep** (`A`) defaults a completed, validated usable episode's outcome to **Success**. Already assessed outcomes and explicit review edits are preserved. Unusable episodes and recordings that fail to complete do not receive this default.

## Recording shortcuts

| Key | Action |
| --- | --- |
| `W` | Start episode |
| `A` | Stop and keep |
| `D` | Stop and mark unusable |

Press the letter without Ctrl or Alt while the collection window is active. Shortcuts follow the corresponding button's enabled state, including readiness checks before starting. They do not activate while editing text, using a dropdown, or interacting with a dialog. Holding a key does not repeat the action. Marking an episode unusable still saves the recording and sets its disposition to `unusable`.

## Recorded topics

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
/laundry_butler/demonstration/command_left
/laundry_butler/demonstration/command_right
/laundry_butler/demonstration/source_left
/laundry_butler/demonstration/source_right
```

The command topics are `sensor_msgs/msg/JointState` with `joint0..5` in radians, `joint6` as the commanded gripper opening in metres, and commanded gripper effort in `effort[6]` (N m). Their header uses the actual kernel CAN receive timestamp. New commands are emitted only when a complete new joint-target cycle or a real gripper command arrives; there is no timer republishing cached targets.

The source topics are `std_msgs/msg/String` JSON records containing the exact four source CAN frames, their individual receive timestamps, and the CAN interface. Joint targets come from `0x155/0x156/0x157` and gripper targets from `0x159`, decoded using the Piper V2 protocol. The receiver uses raw sockets without initializing the Piper SDK or sending CAN frames. `/master/joint_left` and `/master/joint_right` remain excluded because the existing driver refreshes timestamps on cached values.

After recording, validation checks nonzero command counts, gaps and edge coverage, exact seven-value layout, source timestamps, units, and agreement between targets and source frames. A failed new recording is marked unusable. Old observation-only episodes remain readable and receive an explicit warning that they lack independent actions; their bags are not changed.

Independent ROS subscriptions can start/stop one message apart. Unpaired source/target samples within 50 ms of the paired stream's boundary produce a warning and are excluded from `validation.json.command_verified_interval_ns`. Any unpaired interior sample fails validation. Conversion must stay inside that verified common interval and use timestamp-matched source/target samples.

For conversion, use `/puppet/joint_*` for `observation.state` and these independent target streams for `action`, aligned by timestamps. Do not silently copy measured state into action for new episodes. GUI recording does not itself convert or repair existing datasets.

## Output

The normal `./gui/data_collection_gui.sh` launcher saves to the shared SSD:

```text
/srv/aloha-shared/dxx_data/data
```

The launcher's `/home/laundrybutler/Aloha Shared SSD` path is a symlink to
`/srv/aloha-shared`. Set `LAUNDRY_BUTLER_DATA_ROOT` to use a different root.
Running `data_collection_gui.py` directly without this environment variable
falls back to `/home/laundrybutler/laundry-butler/data_collection/output`.

New recordings use MCAP's native **lossless Zstd chunk compression**, with
the settings in `config/recording/mcap_zstd.yaml`. The image pixels, 30 Hz
camera streams, 200 Hz arm streams, topics, and message timestamps are preserved.
Indexes and chunk CRCs remain enabled so existing replay and conversion tools
can read the bags normally. Existing recordings are not rewritten.

To override the storage settings, set `LAUNDRY_BUTLER_MCAP_STORAGE_CONFIG` to
an MCAP writer YAML file before launching. Each episode saves and uses its own
`mcap_storage.yaml` copy, referenced by `episode.json.recording_storage`.
Restart an already-running collection GUI to load this change.

Episode structure:

```text
episode_<timestamp>_<task>/
├── bag/
│   ├── *.mcap
│   └── metadata.yaml
├── episode.json
├── mcap_storage.yaml
├── validation.json
└── recorder.log
```

The MCAP content is not edited. Review properties are written into `episode.json`. “Delete episode” moves the full directory into:

```text
/srv/aloha-shared/dxx_data/data/.trash/
```

## Playback isolation

Episode playback remaps every recorded topic from its original name to:

```text
/laundry_butler/viewer/<original-topic>
```

This prevents playback messages from appearing on the live camera and arm topics. Playback is observation only and never publishes robot command topics.
