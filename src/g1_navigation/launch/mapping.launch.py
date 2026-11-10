"""G1 mapping: the counterpart of home2's
nav_main/launch/task_launch/mapping.launch.py.

slam_toolbox in mapping mode on /scan (the MID-360 projected to 2D by
g1_perception) plus nav_central in mapping mode. With nav2:=true, Nav2 runs on
the growing map too (home2's restaurant-style hybrid), so the map can be built
by sending goals instead of teleop. Save with
    src/g1_navigation/scripts/run_nav.sh --save-map <name>
or nav_ui's "Save Map". Then tag areas (run_nav.sh --tagger) and navigate
(navigation.launch.py map_name:=<name>).

As in navigation.launch.py, sensors and odometry come from the bringup that
includes this file (g1_sim_bringup sim_mapping, or g1_bringup robot).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from g1_navigation.launch_utils import check_mode, merged_nav2_params


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("g1_navigation")
    cfg = os.path.join(get_package_share_directory("g1_nav_overlay"), "config")
    mode = check_mode(LaunchConfiguration("mode").perform(context))
    if mode == "mujoco":
        raise RuntimeError("mode:=mujoco has no lidar to map with; map in gz or on the robot")
    sim_time_arg = LaunchConfiguration("use_sim_time").perform(context)
    use_sim_time = (mode == "gz") if sim_time_arg == "" else sim_time_arg.lower() == "true"
    use_nav2 = LaunchConfiguration("nav2").perform(context).lower() == "true"
    st = {"use_sim_time": use_sim_time}

    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "slam.launch.py")),
        launch_arguments={"slam_mode": "mapping", "use_sim_time": str(use_sim_time).lower()}.items(),
    )]
    if use_nav2:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("nav2_bringup"), "launch", "navigation_launch.py")),
            launch_arguments={
                "params_file": merged_nav2_params(cfg, mode, name=f"nav2_g1_{mode}_mapping_merged.yaml"),
                "use_sim_time": str(use_sim_time).lower(),
                "autostart": "false",
            }.items(),
        ))
    actions.append(Node(
        package="g1_navigation", executable="nav_central.py", name="nav_central", output="screen",
        parameters=[st, {
            "mapping": True,
            "localization": False,
            "use_nav2": use_nav2,
            "map_name": "mapping_session",
            "base_frame": "base_link",
            "required_topics": ["/cmd_vel", "/scan"],
            "required_frames": ["base_link", "mid360_link"],
        }],
    ))
    if LaunchConfiguration("nav_ui").perform(context).lower() == "true":
        actions.append(Node(
            package="g1_navigation", executable="nav_ui.py", name="nav_ui", output="screen",
            parameters=[st, {"mode": "mapping", "map_name": "mapping_session"}],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="gz", description="gz | robot"),
        DeclareLaunchArgument("use_sim_time", default_value="", description="'' -> true for gz only"),
        DeclareLaunchArgument("nav2", default_value="true",
                              description="Run Nav2 on the growing map so goals can drive the mapping"),
        DeclareLaunchArgument("nav_ui", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
