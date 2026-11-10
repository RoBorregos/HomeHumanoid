"""FAST-LIO + lio_odom_adapter: lidar-inertial odometry as odom -> base_link.

Needs the third_party/lio_ws overlay (third_party/fetch_lio.sh), which
provides fast_lio. FAST-LIO's own TF (camera_init -> body) is remapped away;
the adapter re-expresses the pose as odom -> base_link (see its docstring).

  profile:=sim    body-cropped Gazebo cloud, identity extrinsic (fast_lio_sim.yaml)
  profile:=robot  livox_ros_driver2 CustomMsg (fast_lio_robot.yaml)
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    try:
        get_package_share_directory("fast_lio")
    except PackageNotFoundError as e:
        raise RuntimeError(
            "fast_lio not found: run third_party/fetch_lio.sh and "
            "`source third_party/lio_ws/install/setup.bash` before building/launching"
        ) from e

    share = get_package_share_directory("g1_perception")
    profile = LaunchConfiguration("profile").perform(context)
    if profile not in ("sim", "robot"):
        raise RuntimeError(f"profile must be sim or robot, got {profile!r}")
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    publish_tf = LaunchConfiguration("publish_tf").perform(context).lower() == "true"
    seed = {k: float(LaunchConfiguration(k).perform(context)) for k in ("initial_x", "initial_y", "initial_yaw")}

    fast_lio = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fast_lio",
        output="screen",
        parameters=[os.path.join(share, "config", f"fast_lio_{profile}.yaml"), {"use_sim_time": use_sim_time}],
        # Its camera_init -> body TF would be a second, disconnected tree.
        remappings=[("/tf", "/fast_lio/tf"), ("/tf_static", "/fast_lio/tf_static")],
    )
    adapter = Node(
        package="g1_perception",
        executable="lio_odom_adapter.py",
        name="lio_odom_adapter",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "publish_tf": publish_tf,
            "output_topic": LaunchConfiguration("odom_topic").perform(context),
            **seed,
        }],
    )
    return [fast_lio, adapter]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="robot", description="sim | robot"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("publish_tf", default_value="true",
                              description="false to compare against another odom source without owning the TF"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("initial_x", default_value="0.0"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),
        OpaqueFunction(function=launch_setup),
    ])
