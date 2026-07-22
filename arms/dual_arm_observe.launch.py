from launch import LaunchDescription
from launch_ros.actions import Node


ISOLATED_ROOT = "/laundry_butler/observation_only"


def make_arm_node(side: str, can_port: str) -> Node:
    return Node(
        package="piper",
        executable="piper_single_ctrl",
        name=f"piper_{side}_ctrl_node",
        output="screen",
        parameters=[
            {
                "can_port": can_port,
                "auto_enable": False,
                "rviz_ctrl_flag": False,
                "girpper_exist": True,
                "piper_name": side,
            }
        ],
        remappings=[
            (
                "pos_cmd",
                f"{ISOLATED_ROOT}/{side}/pos_cmd",
            ),
            (
                "joint_ctrl_single",
                f"{ISOLATED_ROOT}/{side}/joint_ctrl_single",
            ),
            (
                "enable_flag",
                f"{ISOLATED_ROOT}/{side}/enable_flag",
            ),
            (
                "enable_srv",
                f"{ISOLATED_ROOT}/{side}/enable_srv",
            ),
        ],
    )


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            make_arm_node("left", "can_left"),
            make_arm_node("right", "can_right"),
        ]
    )
