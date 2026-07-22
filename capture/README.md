# Camera capture GUI

A small ROS 2 interface for the Laundry Butler camera workflow.

## Features

- Left, front, and right RGB previews with preserved aspect ratio.
- Per-camera live FPS labels, updated every two seconds.
- Per-camera recording checkboxes.
- Coordinated MCAP recording through `ros2 bag record`.
- PNG snapshots from selected cameras.
- Output grouped into clock-aligned 30-minute folders.
- Active recordings automatically rotate at each half-hour boundary.

## Output

Generated captures go to:

```text
/home/laundrybutler/laundry-butler/capture/output/YYYYMMDD/HHMM-HHMM/
```

The output directory is ignored by Git.

## Dependency check

```bash
source /opt/ros/jazzy/setup.bash
python3 -c 'from PyQt5.QtWidgets import QApplication; print("PyQt5 OK")'
```

Install only if that check fails:

```bash
sudo apt install python3-pyqt5
```

## Run

Launch the RGB cameras first:

```bash
source /opt/ros/jazzy/setup.bash
source /home/laundrybutler/camera_ws/install/setup.bash
export ROS_DOMAIN_ID=88

ros2 launch \
  /home/laundrybutler/laundry-butler/launch/multi_camera_rgb.launch.py
```

Then launch the interface in another terminal:

```bash
cd /home/laundrybutler/laundry-butler
./capture/run_camera_capture_gui.sh
```

The first version records camera topics only. Add robot feedback topics after the Piper observation launch is finalized.
