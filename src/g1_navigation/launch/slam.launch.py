"""slam_toolbox for the G1, mapping or localization, as a lifecycle node that
configures and activates itself (same pattern as home2's
nav_main/launch/omni_setup/{slam,localization}.launch.py).

  slam_mode:=mapping       async_slam_toolbox_node, mapper_params_mapping.yaml
  slam_mode:=localization  localization_slam_toolbox_node against
                           <maps_dir>/<map_name>.posegraph, starting at
                           start_x/start_y/start_yaw

Input is /scan from g1_perception (3D MID-360 projected to 2D), odom ->
base_link from the active odometry source. Publishes map -> odom and /map.
"""

import os

import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

from g1_navigation.paths import maps_dir as default_maps_dir


def launch_setup(context, *args, **kwargs):
    cfg = os.path.join(get_package_share_directory("g1_nav_overlay"), "config")
    slam_mode = LaunchConfiguration("slam_mode").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    maps_dir = LaunchConfiguration("maps_dir").perform(context) or default_maps_dir()

    if slam_mode == "mapping":
        executable = "async_slam_toolbox_node"
        params = [os.path.join(cfg, "mapper_params_mapping.yaml"), {"use_sim_time": use_sim_time}]
    elif slam_mode == "localization":
        executable = "localization_slam_toolbox_node"
        map_name = LaunchConfiguration("map_name").perform(context)
        map_file = os.path.join(maps_dir, map_name)
        if not os.path.exists(map_file + ".posegraph"):
            raise RuntimeError(
                f"{map_file}.posegraph not found: map it first (run_nav.sh --mapping, then --save-map {map_name})"
            )
        start = [float(LaunchConfiguration(k).perform(context)) for k in ("start_x", "start_y", "start_yaw")]
        params = [
            os.path.join(cfg, "mapper_params_localization.yaml"),
            {"use_sim_time": use_sim_time, "map_file_name": map_file, "map_start_pose": start},
        ]
    else:
        raise RuntimeError(f"slam_mode must be mapping or localization, got {slam_mode!r}")

    slam = LifecycleNode(
        package="slam_toolbox",
        executable=executable,
        name="slam_toolbox",
        namespace="",
        output="screen",
        # home2 respawns slam_toolbox so a crash mid-round recovers. In sim
        # that keeps a SIGINT'd launch alive respawning it, so robot only.
        respawn=not use_sim_time,
        respawn_delay=2.0,
        parameters=params,
    )
    configure = RegisterEventHandler(OnProcessStart(
        target_action=slam,
        on_start=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        ))],
    ))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam, start_state="configuring", goal_state="inactive",
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        ))],
    ))
    return [slam, configure, activate]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("slam_mode", default_value="localization", description="mapping | localization"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map_name", default_value=os.environ.get("MAP_NAME", "robocup2026_1")),
        DeclareLaunchArgument("maps_dir", default_value="", description="'' -> G1_MAPS_DIR or the installed maps/"),
        DeclareLaunchArgument("start_x", default_value="0.0"),
        DeclareLaunchArgument("start_y", default_value="0.0"),
        DeclareLaunchArgument("start_yaw", default_value="0.0"),
        OpaqueFunction(function=launch_setup),
    ])
