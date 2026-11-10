"""Track B: Nav2 driving the G1's RL walking policy in unitree_mujoco, through
the G1 navigation stack (g1_navigation, mode:=mujoco).

unitree_mujoco and unitree_rl_lab's g1_ctrl are not ROS 2 nodes, so start them
first (scripts/start_mujoco.sh in this package). This file brings up the ROS 2
side: g1_rl_bridge, robot_state_publisher (+ base_link -> pelvis), then
g1_navigation with a static map and nav_central, so the same
/navigation/go_to_map_area etc. work as in Gazebo and on the robot.

The MuJoCo scene has no arena and no lidar, so the robot walks on an empty plane
while Nav2 plans on the map: this validates cmd_vel -> walking -> odom, not
obstacle avoidance or localization. See nav2_g1_mujoco_overlay.yaml.

odom_source:=leg (default) takes /odom and odom -> base_link from g1_leg_odom's
estimator, from joint angles and the IMU as on the real robot, run inside
g1_rl_bridge. MuJoCo's ground truth then goes to /odom_ground_truth, without TF,
to measure the drift (scripts/odom_error.py). odom_source:=ground_truth is the
original setup, with ground truth on /odom.

Everything runs on wall time: nothing publishes /clock on this track.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    spawn = {k: float(arg(k)) for k in ("spawn_x", "spawn_y", "spawn_yaw")}
    odom_source = arg("odom_source")
    if odom_source not in ("leg", "ground_truth"):
        raise RuntimeError(f"odom_source must be 'leg' or 'ground_truth', got {odom_source!r}")
    dds = {"dds_network_interface": "lo", "dds_domain_id": 0}

    bridge = Node(
        package="g1_rl_bridge",
        executable="g1_rl_bridge",
        output="screen",
        parameters=[spawn, dds, {"odom_source": odom_source}],
    )
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("g1_description"), "launch", "description.launch.py")),
        launch_arguments={"use_sim_time": "false"}.items(),
    )
    actions = [bridge, description]
    if arg("nav").lower() == "true":
        actions.append(TimerAction(period=4.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("g1_navigation"), "launch", "navigation.launch.py")),
            launch_arguments={
                "mode": "mujoco", "map_name": arg("map_name"),
                "start_x": str(spawn["spawn_x"]), "start_y": str(spawn["spawn_y"]),
                "start_yaw": str(spawn["spawn_yaw"]),
            }.items(),
        )]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map_name", default_value=os.environ.get("MAP_NAME", "robocup2026_1")),
        DeclareLaunchArgument("spawn_x", default_value="4.17764"),
        DeclareLaunchArgument("spawn_y", default_value="-11.69743"),
        DeclareLaunchArgument("spawn_yaw", default_value="-1.989"),
        DeclareLaunchArgument("odom_source", default_value="leg", description="leg | ground_truth"),
        DeclareLaunchArgument("nav", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
