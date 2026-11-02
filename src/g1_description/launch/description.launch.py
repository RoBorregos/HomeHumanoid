"""Publishes the G1's robot_state_publisher from the xacro in urdf/.

M2a placeholder: urdf/ currently only has instructions (README.md); g1.urdf.xacro
still needs to be vendored from unitreerobotics/unitree_ros before this launch
file actually works.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("g1_description")
    default_xacro = os.path.join(pkg_share, "urdf", "g1.urdf.xacro")

    xacro_arg = DeclareLaunchArgument(
        "xacro_path",
        default_value=default_xacro,
        description="Path to the G1's xacro (see urdf/README.md)",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command(
                    ["xacro ", LaunchConfiguration("xacro_path")]
                )
            }
        ],
    )

    return LaunchDescription([xacro_arg, robot_state_publisher])
