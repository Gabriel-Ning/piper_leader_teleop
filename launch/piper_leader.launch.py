from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("piper_leader_teleop"))
    config = LaunchConfiguration("config")
    node_name = LaunchConfiguration("node_name")
    leader_model_xacro = LaunchConfiguration("leader_model_xacro")
    leader_robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            leader_model_xacro,
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config", default_value=str(share / "config" / "piper_leader.yaml")
            ),
            DeclareLaunchArgument(
                "node_name",
                default_value="piper_leader",
                description="Node name; use piper_leader_left / piper_leader_right for dual-arm.",
            ),
            DeclareLaunchArgument(
                "leader_model_xacro",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("piper_description"),
                        "urdf",
                        "piper_with_teach.urdf.xacro",
                    ]
                ),
                description="Leader-only Piper + teaching-pendant xacro.",
            ),
            Node(
                package="piper_leader_teleop",
                executable="piper_leader_node",
                name=node_name,
                output="screen",
                parameters=[
                    config,
                    {"leader_robot_description": leader_robot_description},
                ],
            ),
        ]
    )
