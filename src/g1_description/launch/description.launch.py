"""Publishes the G1's robot_state_publisher from the xacro in urdf/, plus the
static base_link -> pelvis transform.

base_link is the planar footprint under the pelvis, on the floor. Odometry
(g1_base_bridge, g1_rl_bridge, g1_loco_bridge, or FAST-LIO through
g1_perception's adapter) publishes odom -> base_link, and the URDF tree hangs
below pelvis, so the full chain is map -> odom -> base_link -> pelvis -> ...
That keeps base_link, the frame every home2 node assumes, without re-rooting
the URDF Gazebo spawns.

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
                ),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    base_height_arg = DeclareLaunchArgument(
        "base_height",
        default_value="0.79",
        description="Pelvis height over the floor in the standing pose (m)",
    )
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")

    base_to_pelvis = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_pelvis",
        arguments=["--frame-id", "base_link", "--child-frame-id", "pelvis",
                   "--z", LaunchConfiguration("base_height")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription(
        [xacro_arg, base_height_arg, use_sim_time_arg, robot_state_publisher, base_to_pelvis]
    )
