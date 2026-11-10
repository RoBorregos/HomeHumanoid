"""MID-360 cloud -> body crop -> /livox/points_filtered -> /scan.

One container with intra-process comms, so the ~23k-point cloud is not
serialized between the two filters. Inputs/outputs:
  cloud_in (default /livox/lidar, PointCloud2 in mid360_link)
  /livox/points_filtered  PointCloud2 in mid360_link, robot body removed
                          (VoxelLayer in the local costmap, FAST-LIO in sim)
  /scan                   LaserScan in base_link, 360 deg, obstacles between
                          min_height and max_height over the floor
                          (slam_toolbox, obstacle layers, nav_central door check)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    params = os.path.join(get_package_share_directory("g1_perception"), "config", "lidar_pipeline.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    common = [params, {"use_sim_time": use_sim_time}]

    container = ComposableNodeContainer(
        name="lidar_pipeline",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        composable_node_descriptions=[
            ComposableNode(
                package="pcl_ros",
                plugin="pcl_ros::CropBox",
                name="body_crop",
                parameters=common,
                remappings=[("input", LaunchConfiguration("cloud_in")),
                            ("output", "/livox/points_filtered")],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="pointcloud_to_laserscan",
                plugin="pointcloud_to_laserscan::PointCloudToLaserScanNode",
                name="cloud_to_scan",
                parameters=common,
                remappings=[("cloud_in", "/livox/points_filtered"), ("scan", "/scan")],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("cloud_in", default_value="/livox/lidar",
                              description="PointCloud2 from the MID-360 (or its sim stand-in)"),
        container,
    ])
