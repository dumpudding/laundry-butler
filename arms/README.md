# Arm observation GUI

Observation-only ROS 2 interface for the Laundry Butler bimanual Piper setup.

## Safety scope

This interface is deliberately non-commanding.

It does not provide controls for:

- Enabling or disabling an arm
- Joint motion
- Cartesian motion
- Gripper motion
- Reset
- Stop
- Homing
- Replay
- Inference

Before starting the Piper nodes:

- Place both arms in the manufacturer-required horizontal/reset pose.
- Manually close both grippers.
- Preserve the labeled LARM and RARM USB/CAN connections.
- Keep the emergency stop accessible.

The project launch sets:

```text
auto_enable=false
rviz_ctrl_flag=false
```

It remaps all position, joint, enable-topic, and enable-service endpoints into an observation-only namespace.

The GUI verifies those endpoints before allowing MCAP recording.

## CAN mapping

| Interface | Adapter serial | Physical role |
|---|---|---|
| `can_left` | `0029001C4148570D20343133` | LARM |
| `can_right` | `003200184148570A20343133` | RARM |

Both interfaces use:

```text
Bitrate: 1000000
TX queue length: 1000
Expected CAN state: ERROR-ACTIVE
```

Do not infer arm roles from temporary names such as `can0` or `can1`.

## Run

Confirm that both CAN interfaces are up:

```bash
ip -brief link show dev can_left
ip -brief link show dev can_right
```

Then launch:

```bash
cd /home/laundrybutler/laundry-butler
./gui/run_arm_status_gui.sh
```

The launcher sources:

```text
/opt/ros/jazzy/setup.bash
/home/laundrybutler/piper_ws/install/setup.bash
```

It defaults to:

```text
ROS_DOMAIN_ID=88
```

## Interface

The GUI provides:

- Start and stop of the tracked observation-only Piper launch.
- `can_left` and `can_right` state.
- CAN bitrate, traffic, error counts, and RX-drop deltas.
- Command-isolation verification.
- Joint feedback sample rate.
- Six joint positions in radians and degrees.
- Raw gripper position in metres.
- Display gripper opening in millimetres.
- Gripper effort feedback.
- Stamped end position and quaternion.
- Arm status, control mode, feedback mode, motion state, teaching state, and error code.
- Per-arm recording selection.
- MCAP start and stop controls.

The GUI refuses to start duplicate Piper nodes. It only stops a Piper launch process that it started itself.

## Command isolation

The observation launch remaps these command topics:

```text
/laundry_butler/observation_only/left/pos_cmd
/laundry_butler/observation_only/left/joint_ctrl_single
/laundry_butler/observation_only/left/enable_flag

/laundry_butler/observation_only/right/pos_cmd
/laundry_butler/observation_only/right/joint_ctrl_single
/laundry_butler/observation_only/right/enable_flag
```

The enable services are remapped to:

```text
/laundry_butler/observation_only/left/enable_srv
/laundry_butler/observation_only/right/enable_srv
```

For command isolation to be marked `VERIFIED`:

- Both Piper nodes must be running.
- All six isolated command topics must have zero publishers.
- All six isolated command topics must have their expected subscribers.
- Both isolated enable services must exist.
- The default enable services must not be exposed.

Do not call the isolated enable services.

## Recorded topics

For each selected arm:

```text
/puppet/joint_<side>
/puppet/end_pose_<side>
/piper_<side>_ctrl_node/arm_status
```

Default full-arm topic set:

```text
/puppet/joint_left
/puppet/joint_right
/puppet/end_pose_left
/puppet/end_pose_right
/piper_left_ctrl_node/arm_status
/piper_right_ctrl_node/arm_status
```

## Joint feedback interpretation

Measured state comes from:

```text
/puppet/joint_left
/puppet/joint_right
```

The message layout is:

```text
position[0:6]   Six arm joints in radians
position[6]     Measured gripper feedback in metres
effort[6]       Gripper effort feedback divided by 1000
```

Small negative gripper positions may appear when the gripper is physically closed. Record the raw value without clamping it.

The GUI may display a second clamped opening value for readability, but the MCAP preserves the raw feedback.

Do not use:

```text
/master/joint_left
/master/joint_right
```

as measured physical state. These topics represent command-state feedback and remain zero when no commands are sent.

## Frame-rate interpretation

The GUI displays its local Python callback/sample rate. This is not the authoritative source or MCAP recording rate.

A short validation recording captured:

```text
Duration:             3.125131 seconds
Left joint:           625 messages
Right joint:          625 messages
Left end pose:        625 messages
Right end pose:       625 messages
Left arm status:      626 messages
Right arm status:     625 messages
```

This is approximately 200 Hz per selected topic.

Use MCAP message counts as the authoritative recording-rate measurement.

## Output

Generated recordings and launch logs are written under:

```text
/home/laundrybutler/laundry-butler/arms/output/
```

This directory is ignored by Git.

## Known vendor behavior

The Piper SDK may print Python 3.12 invalid-escape `SyntaxWarning` messages during startup. These warnings are nonfatal.

`auto_enable=false` prevents the startup enable loop, but it does not inherently make the factory node receive-only. Command and enable endpoints must remain isolated.

The factory Cartesian gripper callback contains a likely unit-clamping issue and must be audited before Cartesian gripper control is enabled.

## Scope

This arm interface remains a subsystem diagnostic and observation-recording tool.

The upcoming unified data-collection interface will record synchronized camera and arm topics into one episode and will provide episode browsing, playback, validation, and human annotation.