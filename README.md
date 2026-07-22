# laundry-butler
Laundry folding task trained on Piper and VLA / RL methods

## Piper vendor workspace and CAN identity

The Piper ROS 2 driver is maintained as a separate vendor workspace at:

```text
/home/laundrybutler/piper_ws
```

It was cloned directly from the original AgileX factory computer and pinned to commit:

```text
e38e0c62319140116ab176a9d1d3c4b51aa6401e
```

The workspace builds successfully under ROS 2 Jazzy and Python 3.12. It uses Piper SDK 0.6.1; the original factory computer used SDK 0.5.0. The factory ROS node imports successfully, and all SDK methods it uses are available on the new system.

The workstation currently detects two USB-to-CAN adapters:

| Current kernel name | Stable serial | Current USB path | Driver | State |
|---|---|---|---|---|
| `can0` | `0029001C4148570D20343133` | `3-7:1.0` | `gs_usb` | DOWN / STOPPED |
| `can1` | `003200184148570A20343133` | `3-11.2:1.0` | `gs_usb` | DOWN / STOPPED |

Both devices identify as `bytewerk candleLight USB-to-CAN adapter`.

Important:

- `can0` and `can1` are temporary kernel enumeration names.
- Left and right arm roles have not yet been physically verified.
- Do not assume `can0` is the mobile chassis.
- Do not copy USB bus paths from the original AgileX computer.
- Bind logical names such as `can_left` and `can_right` using stable adapter serials after the physical left/right mapping is confirmed.
- Do not run `can_config.sh`, `start_multi_piper.sh`, or launch the dual-arm node until the CAN mapping and arm safety checklist are complete.
- The factory dual-arm launch defaults to `auto_enable:=true`.

The vendor workspace, its build products, and host-specific CAN configuration are intentionally kept outside the Laundry Butler Git repository.

<!-- CAMERA-HARDWARE-MAPPING:START -->
## Camera hardware mapping

The three Orbbec Dabai DC1 cameras were identified visually using `ffplay` and
their stable `/dev/v4l/by-id` serial links.

| Role | ROS namespace | Stable serial | Stable color-device link |
|---|---|---|---|
| Front/top camera | `camera_f` | `CC1WC52009R` | `/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._Dabai_DC1_CC1WC52009R-video-index0` |
| Left wrist camera | `camera_l` | `CC1WC52006V` | `/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._Dabai_DC1_CC1WC52006V-video-index0` |
| Right wrist camera | `camera_r` | `CC1WC52012P` | `/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._Dabai_DC1_CC1WC52012P-video-index0` |

Do not persist `/dev/video0`, `/dev/video2`, or `/dev/video4`; numeric video
device names may change after reconnecting devices or rebooting.

The left and right wrist cameras currently share part of the USB topology.
Camera roles must be selected by serial number rather than inferred from USB
enumeration or parent-hub layout.
<!-- CAMERA-HARDWARE-MAPPING:END -->
