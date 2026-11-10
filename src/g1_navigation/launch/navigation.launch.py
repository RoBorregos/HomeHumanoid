"""G1 navigation: the counterpart of home2's
nav_main/launch/task_launch/general_navigation.launch.py.

Brings up the same ROS interface task_manager uses on FRIDA
(/navigation/go_to_map_area, /navigation/areas_json, /navigation/plan_patrol,
...): nav_central, semantic_nav, person_goal_smoother, optionally table_docker
and nav_ui, slam_toolbox localization (or a static map) and Nav2.

Sensors and odometry are NOT started here: g1_sim_bringup (sim_gz /
sim_mujoco) or g1_bringup (robot) start the robot, odom -> base_link and /scan,
then include this file.

  mode:=gz      Gazebo Track A: slam_toolbox localization on /scan, MPPI
  mode:=mujoco  MuJoCo Track B: no lidar, static map + identity map -> odom, RPP
  mode:=robot   real G1: slam_toolbox localization, RPP at loco_bridge limits

Start pose: start_x/start_y/start_yaw, or (empty) the map's
areas_<map>.json start_location.safe_place. nav_central waits for
/initialpose before it reports ready; publish_initial_pose (default true in
sim) sends it, on the robot set it from nav_ui or RViz.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from g1_navigation.launch_utils import check_mode, merged_nav2_params, start_pose_from_areas
from g1_navigation.paths import maps_dir as default_maps_dir


def _bool(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ("true", "1")


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("g1_navigation")
    cfg = os.path.join(get_package_share_directory("g1_nav_overlay"), "config")
    mode = check_mode(LaunchConfiguration("mode").perform(context))
    map_name = LaunchConfiguration("map_name").perform(context).replace(".db", "")
    maps_dir = LaunchConfiguration("maps_dir").perform(context) or default_maps_dir()
    sim_time_arg = LaunchConfiguration("use_sim_time").perform(context)
    use_sim_time = (mode == "gz") if sim_time_arg == "" else sim_time_arg.lower() == "true"
    init_arg = LaunchConfiguration("publish_initial_pose").perform(context)
    publish_initial_pose = (mode != "robot") if init_arg == "" else init_arg.lower() == "true"

    start = [LaunchConfiguration(k).perform(context) for k in ("start_x", "start_y", "start_yaw")]
    if any(v == "" for v in start):
        pose = start_pose_from_areas(maps_dir, map_name)
        if pose is None:
            print(f"[g1 navigation] no start_location in areas_{map_name}.json, starting at (0, 0, 0)")
            pose = (0.0, 0.0, 0.0)
        start = [str(v) if s == "" else s for v, s in zip(pose, start)]
    sx, sy, syaw = (float(v) for v in start)
    print(f"[g1 navigation] mode={mode} map={map_name} maps_dir={maps_dir} "
          f"start=({sx:.2f}, {sy:.2f}, {syaw:.2f}) use_sim_time={use_sim_time}")

    actions = []
    st = {"use_sim_time": use_sim_time}

    # --- localization -------------------------------------------------------
    if mode == "mujoco":
        # unitree_mujoco has no arena and no lidar: Nav2 plans on the static map
        # and g1_rl_bridge's odom is seeded at the start pose, so map == odom.
        map_yaml = os.path.join(maps_dir, f"{map_name}.yaml")
        actions += [
            Node(package="tf2_ros", executable="static_transform_publisher", name="map_to_odom",
                 arguments=["--frame-id", "map", "--child-frame-id", "odom"], parameters=[st]),
            Node(package="nav2_map_server", executable="map_server", name="map_server", output="screen",
                 parameters=[st, {"yaml_filename": map_yaml}]),
            Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                 name="lifecycle_manager_map", output="screen",
                 parameters=[st, {"autostart": True, "node_names": ["map_server"]}]),
        ]
    else:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "slam.launch.py")),
            launch_arguments={
                "slam_mode": "localization", "use_sim_time": str(use_sim_time).lower(),
                "map_name": map_name, "maps_dir": maps_dir,
                "start_x": str(sx), "start_y": str(sy), "start_yaw": str(syaw),
            }.items(),
        ))

    # --- Nav2 (started by nav_central through lifecycle_manager_navigation) -
    nav2_params = merged_nav2_params(cfg, mode)
    follow_params = merged_nav2_params(cfg, mode, extra=("nav2_g1_following.yaml",))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("nav2_bringup"), "launch", "navigation_launch.py")),
        launch_arguments={
            "params_file": nav2_params,
            "use_sim_time": str(use_sim_time).lower(),
            "autostart": LaunchConfiguration("nav2_autostart").perform(context),
        }.items(),
    ))

    # --- home2's navigation nodes, ported --------------------------------
    required_topics = ["/cmd_vel"] if mode == "mujoco" else ["/cmd_vel", "/scan"]
    actions.append(Node(
        package="g1_navigation", executable="nav_central.py", name="nav_central", output="screen",
        parameters=[st, {
            "localization": True,
            "map_name": map_name,
            "areas_map_name": map_name,
            "maps_dir": maps_dir,
            "base_frame": "base_link",
            "required_topics": required_topics,
            "required_frames": ["base_link", "mid360_link"],
        }],
    ))
    if _bool(context, "semantic"):
        actions.append(Node(
            package="g1_navigation", executable="semantic_nav_node.py", name="semantic_nav", output="screen",
            parameters=[os.path.join(share, "config", "semantic_nav.yaml"), st, {"map_name": map_name}],
        ))
    if _bool(context, "follow"):
        actions.append(Node(
            package="g1_navigation", executable="person_goal_smoother.py", name="person_goal_smoother",
            output="screen",
            parameters=[st, {"standard_config_file": nav2_params, "follow_config_file": follow_params}],
        ))
    if _bool(context, "table_docker"):
        actions.append(Node(
            package="g1_navigation", executable="table_docker.py", name="table_docker", output="screen",
            parameters=[os.path.join(share, "config", "table_docker_g1.yaml"), st],
        ))
    if _bool(context, "nav_ui"):
        actions.append(Node(
            package="g1_navigation", executable="nav_ui.py", name="nav_ui", output="screen",
            parameters=[st, {"map_name": map_name}],
        ))
    if publish_initial_pose:
        # After slam_toolbox has loaded the graph; repeated for a while because
        # nav_central only reports ready once it has seen one.
        actions.append(TimerAction(period=8.0, actions=[Node(
            package="g1_base_bridge", executable="initial_pose", name="initial_pose", output="screen",
            parameters=[st, {"x": sx, "y": sy, "yaw": syaw, "publish_duration": 10.0}],
        )]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="gz", description="gz | mujoco | robot"),
        DeclareLaunchArgument("map_name", default_value=os.environ.get("MAP_NAME", "robocup2026_1")),
        DeclareLaunchArgument("maps_dir", default_value="", description="'' -> G1_MAPS_DIR or the installed maps/"),
        DeclareLaunchArgument("use_sim_time", default_value="", description="'' -> true for gz only"),
        DeclareLaunchArgument("start_x", default_value=""),
        DeclareLaunchArgument("start_y", default_value=""),
        DeclareLaunchArgument("start_yaw", default_value=""),
        DeclareLaunchArgument("publish_initial_pose", default_value="", description="'' -> true in sim"),
        DeclareLaunchArgument("nav2_autostart", default_value="false",
                              description="false: nav_central starts Nav2 once its requirements are up (home2 behaviour)"),
        DeclareLaunchArgument("semantic", default_value="true"),
        DeclareLaunchArgument("follow", default_value="true"),
        DeclareLaunchArgument("table_docker", default_value="false",
                              description="Untested on legs: off until tuned (config/table_docker_g1.yaml)"),
        DeclareLaunchArgument("nav_ui", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
