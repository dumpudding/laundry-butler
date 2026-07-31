# Laundry Butler data-collection UI plan

## Current direction

Keep recording observation-only, but combine collection and review in one responsive application. The source MCAP remains unmodified. Review properties live in `episode.json`, and deletion is reversible by moving the full episode directory into `.trash`.

## Collection workflow

1. Start or attach to the three camera nodes.
2. Start or attach to the two observation-only Piper nodes.
3. Confirm all camera and arm observation panels are live.
4. Enter task, garment, initial state, instruction, and optional notes.
5. Select **Start episode**.
6. The application automatically runs readiness checks when the previous result is missing or stale.
7. Stop and keep, or stop and mark unusable.
8. Validation runs automatically.
9. Select the episode to assess outcome, disposition, and notes.
10. Replay the synchronized episode when visual review is required.

The operator field was removed because recordings currently come from one workstation/team and it adds friction without resolving a real ambiguity.

## Implemented in Stage 2

- responsive vertical and horizontal splitters;
- saved geometry and splitter positions;
- scalable bold monospace UI;
- three camera previews;
- left/right arm joint, pose, status, rate, and liveness display;
- duplicate-launch refusal;
- automatic readiness checks;
- synchronized MCAP recording;
- episode metadata and validation sidecars;
- outcome values: not assessed, success, partial, failure;
- dispositions: usable, needs review, unusable;
- editable review notes;
- playback isolated under `/laundry_butler/viewer/*`;
- pause/resume, stop, and rate controls;
- reversible delete-to-trash.

## Why readiness remains

The checks protect against recordings that appear to start but are unusable because a node, topic, CAN link, storage target, or command-isolation condition is wrong. The checks are necessary; forcing a manual preflight click for every episode is not. The application now runs them automatically at startup, after subsystem launch, and immediately before recording when stale.

## Next technical work

### Signal-drop diagnosis

The observed camera and arm streams sometimes disappear midway. Treat this as a separate runtime reliability problem rather than hiding it with UI changes. Capture:

- subsystem logs;
- ROS graph changes;
- topic rates before and after the drop;
- USB resets and kernel messages;
- CAN state/error counters;
- rosbag recorder warnings and message counts.

After the root cause is known, add a recording watchdog that marks the affected interval and blocks a “usable” disposition until reviewed.

### Timeline review

- seek/scrub control;
- synchronized timestamp cursor;
- per-topic dropout markers;
- stage boundaries and excluded ranges;
- versioned `annotations.json`.

### Export

- filter by validation, outcome, and disposition;
- preserve episode provenance;
- convert without modifying MCAPs;
- never include `.trash`.

## Constraints preserved

- ROS 2 Jazzy and default `ROS_DOMAIN_ID=88`;
- human-facing launcher under `gui/`;
- no motion, enable, reset, homing, replay-to-robot, or inference controls;
- `/master/joint_left` and `/master/joint_right` excluded;
- record locally, validate, then copy elsewhere;
- generated output ignored by Git.
