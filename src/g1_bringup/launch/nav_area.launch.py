"""What the navigation container runs (docker/navigation/run.sh), per target.

  mode:=gz      g1_perception's lidar pipeline on Gazebo's MID-360 (+ FAST-LIO
                with odom_source:=lio), then mapping or localization + Nav2 +
                nav_central. Gazebo, the robot and its odometry run in the
                simulation container: sim_gz.launch.py nav:=false perception:=false.
  mode:=mujoco  localization on a static map + Nav2 + nav_central. MuJoCo,
                g1_ctrl and g1_rl_bridge run in the simulation container.
  mode:=robot   robot.launch.py: MID-360 driver, lidar pipeline, FAST-LIO,
                g1_loco_bridge and navigation, i.e. everything that moves the
                real G1, like home2's navigation container with FRIDA's base.

The same nodes and parameters as the all-in-one launch files
(sim_gz.launch.py, sim_mujoco.launch.py, robot.launch.py); only the split
between containers differs.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# The arena spawn pose, as in sim_gz.launch.py / sim_mujoco.launch.py.
SPAWN = {"start_x": "4.17764", "start_y": "-11.69743", "start_yaw": "-1.989"}


def _include(pkg, launch_file, args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(pkg), "launch", launch_file)),
        launch_arguments={k: str(v) for k, v in args.items()}.items(),
    )


def launch_setup(context, *args, **kwargs):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    mode, slam = arg("mode"), arg("slam")
    if mode not in ("gz", "mujoco", "robot"):
        raise RuntimeError(f"mode must be gz, mujoco or robot, got {mode!r}")
    if slam not in ("localization", "mapping"):
        raise RuntimeError(f"slam must be localization or mapping, got {slam!r}")
    start = {k: arg(k) or v for k, v in SPAWN.items()}
    odom = arg("odom_source")

    if mode == "robot":
        args = {k: arg(k) for k in ("network_interface", "host_ip", "lidar_ip", "auto_start",
                                    "map_name", "nav_ui")}
        args.update(slam=slam, odom_source=odom or "lio", mapping_nav2=arg("mapping_nav2") or "false")
        return [_include("g1_bringup", "robot.launch.py", args)]

    if mode == "mujoco":
        if slam == "mapping":
            raise RuntimeError("mujoco has no lidar: map in Gazebo (mode:=gz)")
        return [_include("g1_navigation", "navigation.launch.py",
                         {"mode": "mujoco", "map_name": arg("map_name"), "nav_ui": arg("nav_ui"), **start})]

    odom = odom or "bridge"
    if odom not in ("bridge", "lio"):
        raise RuntimeError(f"odom_source must be bridge or lio in gz, got {odom!r}")
    sensors = [_include("g1_perception", "lidar_pipeline.launch.py",
                        {"use_sim_time": "true", "cloud_in": "/livox/lidar"})]
    if odom == "lio":
        sensors.append(_include("g1_perception", "lio.launch.py", {
            "profile": "sim", "use_sim_time": "true",
            "initial_x": start["start_x"], "initial_y": start["start_y"], "initial_yaw": start["start_yaw"],
        }))
    if slam == "mapping":
        nav = _include("g1_navigation", "mapping.launch.py",
                       {"mode": "gz", "nav2": arg("mapping_nav2") or "true", "nav_ui": arg("nav_ui")})
    else:
        nav = _include("g1_navigation", "navigation.launch.py",
                       {"mode": "gz", "map_name": arg("map_name"), "nav_ui": arg("nav_ui"), **start})
    # The sensors a few seconds before SLAM and Nav2, as in sim_gz.launch.py.
    return [*sensors, TimerAction(period=3.0, actions=[nav])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value=os.environ.get("G1_MODE", "gz"), description="gz | mujoco | robot"),
        DeclareLaunchArgument("slam", default_value="localization", description="localization | mapping"),
        DeclareLaunchArgument("map_name", default_value=os.environ.get("MAP_NAME", "g1_robocup2026_1")),
        DeclareLaunchArgument("odom_source", default_value="",
                              description="'' -> gz: bridge, robot: lio. gz: bridge | lio, robot: lio | leg"),
        DeclareLaunchArgument("mapping_nav2", default_value="",
                              description="slam:=mapping: Nav2 on the growing map ('' -> true in gz, false on the robot)"),
        DeclareLaunchArgument("nav_ui", default_value="false"),
        DeclareLaunchArgument("start_x", default_value=""),
        DeclareLaunchArgument("start_y", default_value=""),
        DeclareLaunchArgument("start_yaw", default_value=""),
        # robot only, see robot.launch.py
        DeclareLaunchArgument("network_interface", default_value="eth0"),
        DeclareLaunchArgument("host_ip", default_value="192.168.123.164"),
        DeclareLaunchArgument("lidar_ip", default_value="192.168.123.120"),
        DeclareLaunchArgument("auto_start", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
