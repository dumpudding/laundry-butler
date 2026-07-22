"""
NOTE THAT THIS LAUNCH FILE IS ONLY RGB CAMERAS, NOT DEPTH CAMERAS.
Orbbec posesses depth info; however, this project will not be using it and is not configured for it.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory  # type: ignore
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription  # type: ignore
from launch.launch_description_sources import PythonLaunchDescriptionSource  # type: ignore


CAMERAS = (  # Working Serial numbers for the Cameras. Reverify after cameras, cables, or USB topology are changed.
    ("camera_f", "CC1WC52009R"),
    ("camera_l", "CC1WC52006V"),
    ("camera_r", "CC1WC52012P"),
)

COMMON_ARGUMENTS = {
    "device_num": "3",
    "enable_color": "true",
    "color_width": "640",
    "color_height": "480",
    "color_fps": "30",
    "color_format": "MJPG",
    "enable_depth": "false",
    "enable_ir": "false",
    "enable_point_cloud": "false",
    "enable_colored_point_cloud": "false",
}


def generate_launch_description():
    dabai_launch = str(
        Path(get_package_share_directory("orbbec_camera"))
        / "launch"
        / "dabai.launch.py"
    )

    cameras = []

    for camera_name, serial_number in CAMERAS:
        arguments = {
            **COMMON_ARGUMENTS,
            "camera_name": camera_name,
            "serial_number": serial_number,
        }

        cameras.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(dabai_launch),
                launch_arguments=arguments.items(),
            )
        )

    return LaunchDescription(cameras)
