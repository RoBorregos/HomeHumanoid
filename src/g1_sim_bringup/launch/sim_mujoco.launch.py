"""Track B: Nav2 driving the G1's RL walking policy in unitree_mujoco.

unitree_mujoco and unitree_rl_lab's g1_ctrl are not ROS 2 nodes, so start them
first (scripts/start_mujoco.sh in this package). This file brings up the ROS 2
side: g1_rl_bridge, a map, and Nav2.

The MuJoCo scene has no arena and no lidar, so the robot walks on an empty plane
while Nav2 plans on the map: this validates cmd_vel -> walking -> odom, not
obstacle avoidance or localization. See nav2_g1_mujoco_overlay.yaml.

odom_source:=leg (default) takes /odom and odom -> pelvis from g1_leg_odom's
estimator, from joint angles and the IMU as on the real robot, run inside
g1_rl_bridge. MuJoCo's ground truth then goes to /odom_ground_truth, without TF,
to measure the drift (scripts/odom_error.py). odom_source:=ground_truth is the
original setup, with ground truth on /odom.

Everything runs on wall time: nothing publishes /clock on this track.
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _deep_merge(base, overlay):
    """Recursively merge overlay dict into base dict (overlay wins)."""
    for k, v in overlay.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _deep_merge(base[k], v)
        else:
            base[k] = v
    return base


def launch_setup(context, *args, **kwargs):
    overlay_share = get_package_share_directory("g1_nav_overlay")
    map_name = LaunchConfiguration("map_name").perform(context)
    spawn = {k: float(LaunchConfiguration(k).perform(context)) for k in ("spawn_x", "spawn_y", "spawn_yaw")}
    odom_source = LaunchConfiguration("odom_source").perform(context)
    if odom_source not in ("leg", "ground_truth"):
        raise RuntimeError(f"odom_source must be 'leg' or 'ground_truth', got {odom_source!r}")
    dds = {"dds_network_interface": "lo", "dds_domain_id": 0}

    with open(os.path.join(overlay_share, "config", "nav2_g1.yaml")) as f:
        params = yaml.safe_load(f)
    with open(os.path.join(overlay_share, "config", "nav2_g1_mujoco_overlay.yaml")) as f:
        _deep_merge(params, yaml.safe_load(f))
    merged = os.path.join(tempfile.gettempdir(), "nav2_g1_mujoco_merged.yaml")
    with open(merged, "w") as f:
        yaml.safe_dump(params, f, default_flow_style=False, sort_keys=False)

    bridge = Node(
        package="g1_rl_bridge",
        executable="g1_rl_bridge",
        output="screen",
        parameters=[spawn, dds, {"odom_source": odom_source}],
    )
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--frame-id", "map", "--child-frame-id", "odom"],
    )
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[{
            "yaml_filename": os.path.join(
                get_package_share_directory("map_context"), "maps", f"{map_name}.yaml"
            ),
        }],
    )
    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[{"autostart": True, "node_names": ["map_server"]}],
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("nav2_bringup"), "launch", "navigation_launch.py")
        ),
        launch_arguments={"params_file": merged, "use_sim_time": "false", "autostart": "true"}.items(),
    )
    return [bridge, map_to_odom, map_server, map_lifecycle, TimerAction(period=4.0, actions=[nav2])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map_name", default_value="robocup2026_1"),
        DeclareLaunchArgument("spawn_x", default_value="4.17764"),
        DeclareLaunchArgument("spawn_y", default_value="-11.69743"),
        DeclareLaunchArgument("spawn_yaw", default_value="-1.989"),
        DeclareLaunchArgument("odom_source", default_value="leg", description="leg | ground_truth"),
        OpaqueFunction(function=launch_setup),
    ])
