"""Track A: the G1 (kinematic virtual base) in Gazebo Harmonic with its 3D
MID-360 stand-in, plus the G1 navigation stack (g1_navigation).

This file starts the "robot": Gazebo, the spawned G1, ros_gz_bridge
(/clock_gz -> clock_throttle -> /clock at 100 Hz, /sim/cmd_vel, /livox/lidar,
/livox/imu), robot_state_publisher +
base_link -> pelvis, the odometry source and g1_perception's lidar pipeline
(/livox/points_filtered, /scan). Then, unless nav:=false, it includes
  slam:=localization  g1_navigation navigation.launch.py mode:=gz (default)
  slam:=mapping       g1_navigation mapping.launch.py mode:=gz

odom_source:=bridge (default) is g1_base_bridge's dead reckoning of the
commanded velocity, exact on this kinematic base. odom_source:=lio runs
FAST-LIO on the simulated cloud + IMU instead (needs third_party/lio_ws), and
the bridge's odometry goes to /odom_bridge for comparison.

Requires src/g1_description/scripts/vendor_g1.sh (G1 URDF/meshes) and the
vendor/home2 packages built (frida_gz_sim world, frida_interfaces).
Defaults to robocup2026_1: home2's world, with the map and areas copied into
g1_navigation/maps.
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
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


def _frida_gz_sim_dir():
    """frida_gz_sim's share dir, or its source in vendor/home2 when it isn't built.

    Only its worlds are used, and their SDFs are self-contained. The simulation
    container doesn't build it: its manifest needs home2's manipulation and
    vision packages (docker/simulation/run.sh).
    """
    try:
        return get_package_share_directory("frida_gz_sim")
    except PackageNotFoundError:
        repo = os.environ.get("G1_REPO") or os.path.abspath(
            os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "..", ".."))
        return os.path.join(repo, "vendor", "home2", "simulation", "frida_gz_sim")


def _resource_path():
    paths = [os.path.join(_frida_gz_sim_dir(), "worlds")]
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        share = os.path.join(prefix, "share")
        if os.path.isdir(share):
            paths.append(share)
    extra = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if extra:
        paths.append(extra)
    return os.pathsep.join(paths)


def _include(pkg, launch_file, args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(pkg), "launch", launch_file)),
        launch_arguments=args.items(),
    )


def launch_setup(context, *args, **kwargs):
    nav_overlay_share = get_package_share_directory("g1_nav_overlay")
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731

    gui = arg("gui").lower() == "true"
    start_gazebo = arg("start_gazebo").lower() == "true"
    spawn = {k: arg(k) for k in ("spawn_x", "spawn_y", "spawn_z", "spawn_yaw")}
    odom_source = arg("odom_source")
    if odom_source not in ("bridge", "lio"):
        raise RuntimeError(f"odom_source must be bridge or lio, got {odom_source!r}")
    slam = arg("slam")
    if slam not in ("localization", "mapping"):
        raise RuntimeError(f"slam must be localization or mapping, got {slam!r}")

    gazebo_actions = []
    if start_gazebo:
        gz_cmd = ["gz", "sim", "-r", "-v", "3", arg("world")]
        if not gui:
            gz_cmd[2:2] = ["-s", "--headless-rendering"]
        # DISPLAY unset + EGL_PLATFORM=surfaceless keeps Ogre2 off the X11
        # EGL platform, which fails on this Orin ("failed to create dri2
        # screen"). The ~0.003 real_time_factor once blamed on that was the
        # robot spawned at z=0 with its legs in the floor (docs/plan.md):
        # RTF is ~1.0 at spawn_z 0.79, also with the 3D lidar (2026-11-10).
        gz_env = dict(os.environ)
        gz_env.pop("DISPLAY", None)
        gz_env["GZ_SIM_RESOURCE_PATH"] = _resource_path()
        gz_env["EGL_PLATFORM"] = "surfaceless"
        gazebo_actions.append(ExecuteProcess(cmd=gz_cmd, output="screen", env=gz_env))

    description = _include("g1_description", "description.launch.py", {"use_sim_time": "true"})
    spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description", "-name", "g1",
            "-x", spawn["spawn_x"], "-y", spawn["spawn_y"], "-z", spawn["spawn_z"], "-Y", spawn["spawn_yaw"],
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
    # Gazebo's /clock (every step) -> /clock at 100 Hz, see clock_throttle.cpp.
    clock_throttle = Node(package="g1_sim_bringup", executable="clock_throttle", output="screen")
    use_lio = odom_source == "lio"
    base_bridge = Node(
        package="g1_base_bridge",
        executable="g1_base_bridge",
        name="g1_base_bridge",
        output="screen",
        parameters=[{
            "backend": "gazebo",
            "use_sim_time": True,
            "spawn_x": float(spawn["spawn_x"]),
            "spawn_y": float(spawn["spawn_y"]),
            "spawn_yaw": float(spawn["spawn_yaw"]),
            "publish_tf": not use_lio,
            "odom_out_topic": "/odom_bridge" if use_lio else "/odom",
        }],
    )
    robot = [bridge, clock_throttle, base_bridge]
    # perception:=false leaves the lidar pipeline (and FAST-LIO) to the navigation
    # container, g1_bringup/nav_area.launch.py (docker/navigation).
    with_perception = arg("perception").lower() == "true"
    if with_perception:
        robot.append(_include("g1_perception", "lidar_pipeline.launch.py",
                              {"use_sim_time": "true", "cloud_in": "/livox/lidar"}))
    if use_lio and with_perception:
        robot.append(_include("g1_perception", "lio.launch.py", {
            "profile": "sim", "use_sim_time": "true",
            "initial_x": spawn["spawn_x"], "initial_y": spawn["spawn_y"], "initial_yaw": spawn["spawn_yaw"],
        }))

    nav = []
    if arg("nav").lower() == "true":
        if slam == "mapping":
            nav.append(_include("g1_navigation", "mapping.launch.py",
                                {"mode": "gz", "nav2": arg("mapping_nav2"), "nav_ui": arg("nav_ui")}))
        else:
            nav.append(_include("g1_navigation", "navigation.launch.py", {
                "mode": "gz", "map_name": arg("map_name"), "nav_ui": arg("nav_ui"),
                "start_x": spawn["spawn_x"], "start_y": spawn["spawn_y"], "start_yaw": spawn["spawn_yaw"],
            }))

    # Staggered startup: gives gazebo a few seconds to load the world before
    # spawn/bridge, and the sensors a few more before SLAM and Nav2.
    return [
        *gazebo_actions,
        TimerAction(period=4.0, actions=[description, spawn_node]),
        TimerAction(period=6.0, actions=robot),
        TimerAction(period=9.0, actions=nav),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument(
            "start_gazebo", default_value="true",
            description="Start gz sim from this launch file. false attaches to a Gazebo you started yourself "
                        "(DISPLAY unset, EGL_PLATFORM=surfaceless).",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(
                _frida_gz_sim_dir(), "worlds", "arena_robocup2026_1.sdf"
            ),
        ),
        DeclareLaunchArgument("map_name", default_value=os.environ.get("MAP_NAME", "robocup2026_1"),
                              description="Map in g1_navigation's maps dir to localize against"),
        DeclareLaunchArgument("slam", default_value="localization", description="localization | mapping"),
        DeclareLaunchArgument("nav", default_value="true",
                              description="false: robot + sensors only (run g1_navigation yourself)"),
        DeclareLaunchArgument("mapping_nav2", default_value="true", description="slam:=mapping: Nav2 on the growing map"),
        DeclareLaunchArgument("nav_ui", default_value="false"),
        DeclareLaunchArgument("perception", default_value="true",
                              description="Lidar pipeline (+ FAST-LIO with odom_source:=lio) here; false: in the navigation container"),
        DeclareLaunchArgument("odom_source", default_value="bridge", description="bridge | lio"),
        # Defaults match robocup2026_1's start_location.safe_place in
        # g1_navigation/maps/areas/areas_robocup2026_1.json.
        DeclareLaunchArgument("spawn_x", default_value="4.17764"),
        DeclareLaunchArgument("spawn_y", default_value="-11.69743"),
        DeclareLaunchArgument(
            "spawn_z", default_value="0.79",
            description="Pelvis height with the legs locked straight. Must keep the feet above the floor: "
                        "at 0.0 the merged leg meshes are buried and every physics step costs ~290ms.",
        ),
        DeclareLaunchArgument("spawn_yaw", default_value="-1.989"),
        OpaqueFunction(function=launch_setup),
    ])
