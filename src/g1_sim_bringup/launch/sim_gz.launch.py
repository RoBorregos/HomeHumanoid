"""Track A: G1 (kinematic virtual base) in Gazebo Harmonic + Nav2, localizing
against a pre-built map. Fase 4 of docs/plan.md — the autonomous-goal-reaching
flow, built on the same world/spawn/bridge/Nav2 foundation sim_mapping.launch.py
(Fase 2) uses.

Requires `./src/g1_description/scripts/vendor_g1.sh` to have been run (G1
URDF/meshes) and `vendor/home2` to be a built submodule (nav_main,
frida_gz_sim, map_context — see docs/plan.md's prerequisite section).

Defaults everything to robocup2026_1 (vendored from home2, world + map +
start pose all already exist) as the first smoke test — pass map_name:=
and world:= to point at a different map once Fase 2/3 produce one for G1.
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _deep_merge(base, overlay):
    """Recursively merge overlay dict into base dict (overlay wins).

    Ported from vendor/home2/navigation/packages/nav_main/launch/omni_setup/
    nav2_omni.launch.py — same pattern, applied here before handing the
    result to stock nav2_bringup's navigation_launch.py instead of a
    composable-node container (see docs/plan.md for why).
    """
    for k, v in overlay.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _deep_merge(base[k], v)
        else:
            base[k] = v
    return base


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
    map_context_share = get_package_share_directory("map_context")

    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    start_gazebo = LaunchConfiguration("start_gazebo").perform(context).lower() == "true"
    world = LaunchConfiguration("world").perform(context)
    map_name = LaunchConfiguration("map_name").perform(context)
    spawn_x = LaunchConfiguration("spawn_x").perform(context)
    spawn_y = LaunchConfiguration("spawn_y").perform(context)
    spawn_z = LaunchConfiguration("spawn_z").perform(context)
    spawn_yaw = LaunchConfiguration("spawn_yaw").perform(context)

    gazebo_actions = []
    if start_gazebo:
        gz_cmd = ["gz", "sim", "-r", "-v", "3", world]
        if not gui:
            gz_cmd[2:2] = ["-s", "--headless-rendering"]
        # DISPLAY unset + EGL_PLATFORM=surfaceless keeps Ogre2 off the X11
        # EGL platform, which fails on this Orin ("failed to create dri2
        # screen"). The ~0.003 real_time_factor once blamed on that, and on
        # running gz sim inside this launch tree, was the robot spawned at
        # z=0 with its legs buried in the floor (docs/plan.md, "M2a continued
        # — third Orin session"): RTF is ~1.0 from here at spawn_z 0.79.
        gz_env = dict(os.environ)
        gz_env.pop("DISPLAY", None)
        gz_env["GZ_SIM_RESOURCE_PATH"] = _resource_path()
        gz_env["EGL_PLATFORM"] = "surfaceless"
        gazebo_actions.append(ExecuteProcess(cmd=gz_cmd, output="screen", env=gz_env))

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
        parameters=[{
            "backend": "gazebo",
            "use_sim_time": True,
            "spawn_x": float(spawn_x),
            "spawn_y": float(spawn_y),
            "spawn_yaw": float(spawn_yaw),
        }],
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_main_share, "launch", "omni_setup", "localization.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": os.path.join(nav_overlay_share, "config", "mapper_params_sim.yaml"),
            "map": os.path.join(map_context_share, "maps", map_name),
            "use_static_map_server": "false",
        }.items(),
    )

    # Deep-merge nav2_g1.yaml + nav2_g1_sim_overlay.yaml into one tempfile,
    # then hand it to stock nav2_bringup (autostart:true — no nav_central.py
    # lifecycle trigger needed, see docs/plan.md).
    with open(os.path.join(nav_overlay_share, "config", "nav2_g1.yaml")) as f:
        merged_params = yaml.safe_load(f)
    with open(os.path.join(nav_overlay_share, "config", "nav2_g1_sim_overlay.yaml")) as f:
        _deep_merge(merged_params, yaml.safe_load(f))
    merged_path = os.path.join(tempfile.gettempdir(), "nav2_g1_merged.yaml")
    with open(merged_path, "w") as f:
        yaml.safe_dump(merged_params, f, default_flow_style=False, sort_keys=False)

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("nav2_bringup"), "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "params_file": merged_path,
            "use_sim_time": "true",
            "autostart": "true",
        }.items(),
    )

    initial_pose = Node(
        package="g1_base_bridge",
        executable="initial_pose",
        output="screen",
        parameters=[{"x": float(spawn_x), "y": float(spawn_y), "yaw": float(spawn_yaw)}],
    )

    # Staggered startup: gives gazebo a few seconds to finish loading the
    # world before spawn/bridge/slam/nav2 start.
    return [
        *gazebo_actions,
        TimerAction(period=4.0, actions=[description_launch, spawn]),
        TimerAction(period=6.0, actions=[bridge, base_bridge]),
        TimerAction(period=8.0, actions=[localization]),
        TimerAction(period=12.0, actions=[nav2]),
        # Give slam_toolbox + nav2's lifecycle manager time to autostart
        # before seeding the initial pose (empirical delay, not tied to any
        # specific startup-sequencing mechanism since there's no nav_central
        # gate here — see docs/plan.md).
        TimerAction(period=20.0, actions=[initial_pose]),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument(
                "start_gazebo",
                default_value="true",
                description="Start gz sim from this launch file. Set false to attach to a "
                            "Gazebo you started yourself (DISPLAY unset, "
                            "EGL_PLATFORM=surfaceless).",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(
                    get_package_share_directory("frida_gz_sim"), "worlds", "arena_robocup2026_1.sdf"
                ),
            ),
            DeclareLaunchArgument(
                "map_name",
                default_value="robocup2026_1",
                description="Name under map_context/maps/ (no extension) to localize against.",
            ),
            # Defaults match robocup2026_1's start_location.safe_place pose
            # (see mapper_params_sim.yaml's map_start_pose) — keep all three
            # in sync if this changes.
            DeclareLaunchArgument("spawn_x", default_value="4.17764"),
            DeclareLaunchArgument("spawn_y", default_value="-11.69743"),
            DeclareLaunchArgument(
                "spawn_z", default_value="0.79",
                description="Pelvis height with the legs locked straight. Must keep the feet above the floor: at 0.0 the merged leg meshes are buried and every physics step costs ~290ms (real_time_factor ~0.003).",
            ),
            DeclareLaunchArgument("spawn_yaw", default_value="-1.989"),
            OpaqueFunction(function=launch_setup),
        ]
    )
