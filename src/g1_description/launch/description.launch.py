"""Publishes the G1's robot_state_publisher from the xacro in urdf/.

Run src/g1_description/scripts/vendor_g1.sh first — g1.urdf.xacro includes
the vendored g1_29dof_raw.urdf, which is gitignored (see urdf/README.md).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
                "robot_description": ParameterValue(
                    Command(["xacro ", LaunchConfiguration("xacro_path")]),
                    value_type=str,
                )
            }
        ],
    )

    return LaunchDescription([xacro_arg, robot_state_publisher])
