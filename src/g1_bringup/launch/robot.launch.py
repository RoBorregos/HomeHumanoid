"""Real G1: sensors, odometry, locomotion bridge and the G1 navigation stack.

UNTESTED ON HARDWARE. First runs with the robot on its gantry and the
wireless remote's emergency stop in hand; put the robot in walking mode with
the remote first (g1_loco_bridge does not change the FSM unless auto_start).

  robot_state_publisher + base_link -> pelvis      (g1_description)
  livox_ros_driver2 -> /livox/lidar, /livox/imu    (head MID-360)
  g1_perception lidar pipeline -> /livox/points_filtered, /scan
  odom_source:=lio  FAST-LIO -> odom -> base_link; g1_loco_bridge's leg
                    estimate goes to /odom_leg (no TF) for comparison
  odom_source:=leg  g1_loco_bridge's leg-inertial odometry owns odom -> base_link
  g1_loco_bridge    /cmd_vel -> LocoClient::SetVelocity (clamped, dead-man)
  nav:=true         g1_navigation navigation.launch.py mode:=robot (default),
                    or mapping.launch.py mode:=robot with slam:=mapping

The lidar driver needs this computer on the G1's 192.168.123.0/24 network
(host_ip) and the MID-360 at lidar_ip; the Unitree DDS side uses
network_interface. livox_ros_driver2 and fast_lio come from
third_party/fetch_lio.sh (source third_party/lio_ws/install/setup.bash).

To verify on the robot before trusting the map (docs/plan.md, M5):
  * the MID-360 mount: g1_description models it upside down (vertical FOV
    -52..+7 deg). With the robot standing still, /livox/imu should read
    gravity along +z if upright, -z if inverted, and the URDF's mid360_joint
    must match what the cloud shows.
  * the crop box in g1_perception/config/lidar_pipeline.yaml against the real
    arm pose (the cloud must not contain the robot's own arms).
"""

import os
import tempfile

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _has(pkg):
    try:
        get_package_share_directory(pkg)
        return True
    except PackageNotFoundError:
        return False


def _include(pkg, launch_file, args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(pkg), "launch", launch_file)),
        launch_arguments=args.items(),
    )


def launch_setup(context, *args, **kwargs):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    odom_source = arg("odom_source")
    if odom_source not in ("lio", "leg"):
        raise RuntimeError(f"odom_source must be lio or leg, got {odom_source!r}")
    use_lio = odom_source == "lio"
    slam = arg("slam")
    if slam not in ("localization", "mapping"):
        raise RuntimeError(f"slam must be localization or mapping, got {slam!r}")

    missing = [p for p in (["livox_ros_driver2"] + (["fast_lio"] if use_lio else [])) if not _has(p)]
    if missing:
        raise RuntimeError(
            f"{', '.join(missing)} not found: run third_party/fetch_lio.sh and source "
            "third_party/lio_ws/install/setup.bash before building/launching"
        )

    # livox_ros_driver2 reads IPs from a JSON file: fill in host/lidar IPs.
    template = os.path.join(get_package_share_directory("g1_bringup"), "config", "MID360_g1.json")
    with open(template) as f:
        cfg = f.read().replace("HOST_IP", arg("host_ip")).replace("LIDAR_IP", arg("lidar_ip"))
    lidar_cfg = os.path.join(tempfile.gettempdir(), "g1_mid360_config.json")
    with open(lidar_cfg, "w") as f:
        f.write(cfg)

    actions = [_include("g1_description", "description.launch.py", {"use_sim_time": "false"})]

    actions.append(Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[{
            # 1 = CustomMsg with per-point time, needed by FAST-LIO's deskew;
            # 0 = PointCloud2 straight to the pipeline when FAST-LIO is off.
            "xfer_format": 1 if use_lio else 0,
            "multi_topic": 0,
            "data_src": 0,
            "publish_freq": 10.0,
            "output_data_type": 0,
            "frame_id": "mid360_link",
            "user_config_path": lidar_cfg,
            "cmdline_input_bd_code": "livox0000000001",
        }],
    ))
    if use_lio:
        actions.append(Node(
            package="g1_perception", executable="livox_custom_to_pc2", name="livox_custom_to_pc2",
            output="screen",
            parameters=[{"input_topic": "/livox/lidar", "output_topic": "/livox/points", "frame_id": "mid360_link"}],
        ))
    actions.append(_include("g1_perception", "lidar_pipeline.launch.py", {
        "use_sim_time": "false", "cloud_in": "/livox/points" if use_lio else "/livox/lidar",
    }))
    if use_lio:
        actions.append(_include("g1_perception", "lio.launch.py", {"profile": "robot", "use_sim_time": "false"}))

    actions.append(Node(
        package="g1_loco_bridge",
        executable="g1_loco_bridge",
        name="g1_loco_bridge",
        output="screen",
        parameters=[{
            "dds_network_interface": arg("network_interface"),
            "auto_start": arg("auto_start").lower() == "true",
            "publish_tf": not use_lio,
            "odom_topic": "/odom_leg" if use_lio else "/odom",
        }],
    ))

    if arg("nav").lower() == "true":
        if slam == "mapping":
            nav = _include("g1_navigation", "mapping.launch.py",
                           {"mode": "robot", "nav2": arg("mapping_nav2"), "nav_ui": arg("nav_ui")})
        else:
            nav = _include("g1_navigation", "navigation.launch.py",
                           {"mode": "robot", "map_name": arg("map_name"), "nav_ui": arg("nav_ui")})
        # Let the driver and FAST-LIO initialise (it needs a few still seconds
        # of IMU) before SLAM and Nav2 look for odom -> base_link.
        actions.append(TimerAction(period=5.0, actions=[nav]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("odom_source", default_value="lio", description="lio | leg"),
        DeclareLaunchArgument("network_interface", default_value="eth0",
                              description="Interface on the G1's 192.168.123.0/24 network (Unitree DDS)"),
        DeclareLaunchArgument("host_ip", default_value="192.168.123.164",
                              description="This computer's IP on the G1 network (PC2 is .164)"),
        DeclareLaunchArgument("lidar_ip", default_value="192.168.123.120", description="The head MID-360"),
        DeclareLaunchArgument("auto_start", default_value="false",
                              description="g1_loco_bridge calls LocoClient::Start(); check by hand first"),
        DeclareLaunchArgument("nav", default_value="true"),
        DeclareLaunchArgument("slam", default_value="localization", description="localization | mapping"),
        DeclareLaunchArgument("mapping_nav2", default_value="false",
                              description="slam:=mapping: Nav2 on the growing map (teleop with the remote by default)"),
        DeclareLaunchArgument("map_name", default_value=os.environ.get("MAP_NAME", "robocup2026_1")),
        DeclareLaunchArgument("nav_ui", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
