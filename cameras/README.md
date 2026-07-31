# Camera capture GUI

ROS 2 camera diagnostics and recording interface for the Laundry Butler project.

## Camera mapping

| Role | ROS namespace | Serial |
|---|---|---|
| Front/top | `camera_f` | `CC1WC52009R` |
| Left wrist | `camera_l` | `CC1WC52006V` |
| Right wrist | `camera_r` | `CC1WC52012P` |

Camera roles are selected by serial number. Do not persist `/dev/videoN` names because numeric device assignments may change after reconnecting cameras or rebooting.

## Current stream configuration

```text
Resolution: 640 × 480
Frame rate: 30 FPS
Format: MJPG
Colour: enabled
Depth: disabled
Infrared: disabled
Point cloud: disabled
```

Depth is intentionally excluded from the current OpenPI π0.5 training path.

## Run

```bash
cd /home/laundrybutler/laundry-butler
./gui/run_camera_capture_gui.sh
```

The launcher sources:

```text
/opt/ros/jazzy/setup.bash
/home/laundrybutler/camera_ws/install/setup.bash
```

It defaults to:

```text
ROS_DOMAIN_ID=88
```

## Interface

The GUI provides:

- Start and stop of the tracked three-camera ROS launch.
- Left, front, and right previews.
- Aspect-ratio-preserving image scaling.
- Per-camera recording selection.
- MCAP start and stop controls.
- Snapshot capture.
- Separate stream and GUI preview frame-rate displays.
- Clock-aligned 30-minute output folders.

The GUI refuses to start duplicate camera nodes when an existing camera launch is already active. It only stops a launch process that it started itself.

## Frame-rate interpretation

The interface shows two different rates:

```text
Stream FPS
View FPS
```

`Stream FPS` is measured using lightweight `camera_info` messages and reflects the camera publication cadence.

`View FPS` is the intentionally reduced GUI preview rate. It may remain around 7–10 FPS while the camera stream and MCAP recording continue near 30 FPS.

Use MCAP message counts as the authoritative recording-rate measurement.

## Recorded topics

For each selected camera:

```text
/camera_<role>/color/image_raw
/camera_<role>/color/camera_info
```

For all three cameras:

```text
/camera_f/color/image_raw
/camera_f/color/camera_info
/camera_l/color/image_raw
/camera_l/color/camera_info
/camera_r/color/image_raw
/camera_r/color/camera_info
```

## Validation

A simultaneous three-camera test recorded approximately:

```text
Front image:       29.9 Hz
Left image:        29.9 Hz
Right image:       29.5 Hz
```

A later GUI-created 17.33-second MCAP recorded:

```text
Front image: 505 messages
Left image:  519 messages
Right image: 519 messages
```

The saved recording remained near 30 FPS even when the GUI preview displayed a substantially lower rate.

Use the following camera-health checks:

1. `camera_info` cadence.
2. MCAP message counts divided by duration.
3. Camera launch logs for decode, reconnect, or device failures.

Do not use `ros2 topic hz` on `sensor_msgs/Image` as the primary health check. Python deserialization of large raw image messages can substantially under-report the real stream rate.

## Output

Generated recordings, snapshots, and runtime logs are written under:

```text
/home/laundrybutler/laundry-butler/cameras/output/
```

Typical layout:

```text
cameras/output/
└── YYYYMMDD/
    └── HHMM-HHMM/
        └── recording-HHMMSS/
            ├── bag/
            │   ├── bag_0.mcap
            │   └── metadata.yaml
            ├── snapshots/
            └── rosbag.log
```

This directory is ignored by Git.

## Scope

This camera interface is a subsystem diagnostic and camera-only recording tool.

The unified data-collection interface now records synchronized camera and observation-only arm topics into one episode. Use this camera interface for camera diagnostics, snapshots, and isolated camera recordings.
