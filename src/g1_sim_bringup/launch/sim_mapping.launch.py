"""Track A: G1 (kinematic virtual base) in Gazebo Harmonic + slam_toolbox in
MAPPING mode. Fase 2 of docs/plan.md — drive G1 around (teleop, in a separate
terminal: `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args
-r cmd_vel:=/cmd_vel`, or any other /cmd_vel source) while this builds a new
map, then save it:

    mkdir -p /tmp/g1_maps
    ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
      "{name: {data: '/tmp/g1_maps/<name>'}}"
    ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
      "{filename: '/tmp/g1_maps/<name>'}"

No Nav2, no initial_pose — this is SLAM only, same world/spawn/bridge
foundation as sim_gz.launch.py (Fase 4). Requires the same prerequisites
(vendor_g1.sh run, vendor/home2 built) — see docs/plan.md.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resource_path():
    paths = [
        os.path.join(get_package_share_directory("frida_gz_sim"), "worlds"),
    ]
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        share = os.path.join(prefix, "share")
        if os.path.isdir(share):
            paths.append(share)
    extra = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if extra:
        paths.append(extra)
    return os.pathsep.join(paths)


def launch_setup(context, *args, **kwargs):
    nav_overlay_share = get_package_share_directory("g1_nav_overlay")
    nav_main_share = get_package_share_directory("nav_main")

    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    world = LaunchConfiguration("world").perform(context)
    spawn_x = LaunchConfiguration("spawn_x").perform(context)
    spawn_y = LaunchConfiguration("spawn_y").perform(context)
    spawn_z = LaunchConfiguration("spawn_z").perform(context)
    spawn_yaw = LaunchConfiguration("spawn_yaw").perform(context)

    gz_cmd = ["gz", "sim", "-r", "-v", "3", world]
    if not gui:
        gz_cmd[2:2] = ["-s", "--headless-rendering"]
    gazebo = ExecuteProcess(
        cmd=gz_cmd,
        output="screen",
        additional_env={"GZ_SIM_RESOURCE_PATH": _resource_path()},
    )

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("g1_description"), "launch", "description.launch.py"
            )
        )
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "g1",
            "-x", spawn_x, "-y", spawn_y, "-z", spawn_z, "-Y", spawn_yaw,
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[
            {"config_file": os.path.join(nav_overlay_share, "config", "bridge_gz.yaml")},
            {"use_sim_time": True},
        ],
    )

    base_bridge = Node(
        package="g1_base_bridge",
        executable="g1_base_bridge",
        name="g1_base_bridge",
        output="screen",
        parameters=[{"backend": "gazebo"}],
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_main_share, "launch", "omni_setup", "slam.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": os.path.join(nav_overlay_share, "config", "mapper_params_mapping.yaml"),
        }.items(),
    )

    return [gazebo, description_launch, spawn, bridge, base_bridge, slam]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(
                    get_package_share_directory("frida_gz_sim"), "worlds", "arena_robocup2026_1.sdf"
                ),
                description="Gazebo world to map. Defaults to home2's robocup2026_1 arena "
                            "(known-good geometry) for a first pass; swap for any other "
                            "world to map a new space.",
            ),
            DeclareLaunchArgument("spawn_x", default_value="4.17764"),
            DeclareLaunchArgument("spawn_y", default_value="-11.69743"),
            DeclareLaunchArgument(
                "spawn_z", default_value="0.0",
                description="TODO: tune once the locked-joint idle stance is visually confirmed (see g1_description).",
            ),
            DeclareLaunchArgument("spawn_yaw", default_value="-1.989"),
            OpaqueFunction(function=launch_setup),
        ]
    )
