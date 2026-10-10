# Unified data-collection GUI — Stage 2

This interface records, reviews, and replays synchronized Laundry Butler demonstrations.

## Implemented

- responsive split-pane layout that scales from 1100×700 upward;
- saved window and splitter geometry;
- episode list using half the review pane by default, with a wider draggable divider and scrollable review/playback controls;
- Roboto Mono Bold request with a monospace fallback when the font is unavailable;
- three reduced-rate camera previews;
- live left/right arm observation for joints, end pose, status, rate, and liveness;
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
- all 12 required recording topics;
- receipt of fresh camera-info, joint, and arm-status samples;
- `can_left` and `can_right` are UP, ERROR-ACTIVE, and at 1 Mbit/s;
- observation-only command topics and enable services remain isolated;
- enough free storage exists at the configured output path.

The checks are still necessary because a rosbag process can start successfully while recording missing topics or writing to an unsuitable disk. The user no longer needs to press the readiness button before every episode: the GUI rechecks automatically when required.

The GUI checks the most recent camera-info, joint, and arm-status samples received by its existing live subscriptions. All seven topics must have a sample no more than 3 seconds old. Playback samples do not count. This avoids creating temporary `ros2 topic echo` subscriptions that can time out during discovery while the live previews continue updating. Freshness is checked again immediately before recording, even when the other readiness checks are still valid. Standalone checks retain the CLI subscription fallback.

## Workflow

1. Put both arms in the manufacturer-required horizontal/reset pose.
2. Manually close both grippers.
3. Preserve the labeled LARM and RARM USB/CAN connections.
4. Keep the emergency stop accessible.
5. Start or attach to cameras and observation-only arms.
6. Confirm the camera and arm observation panels.
7. Fill in the episode metadata.
8. Select **Start episode**. The GUI runs readiness automatically when the previous result is stale.
9. Stop and keep, or stop and mark unusable.
10. Select the episode to edit outcome, disposition, and notes.
11. Play the episode to review synchronized camera and arm topics.

Under the current collection policy, **Stop and keep** (`A`) defaults a completed, usable episode's outcome to **Success**. Already assessed outcomes and explicit review edits are preserved. Unusable episodes and recordings that fail to complete do not receive this default.

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
```

`/master/joint_left` and `/master/joint_right` remain excluded.

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
