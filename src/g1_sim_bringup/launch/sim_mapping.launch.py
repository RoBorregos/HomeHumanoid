"""Track A mapping: sim_gz.launch.py with slam:=mapping.

Builds a new map of the Gazebo world with the 3D MID-360 stand-in (projected to
/scan by g1_perception) and slam_toolbox, with Nav2 running on the growing map
so goals can drive the exploration (mapping_nav2:=false for teleop only).
Save with `src/g1_navigation/scripts/run_nav.sh --save-map <name>`.
All other arguments are sim_gz.launch.py's.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("g1_sim_bringup"), "launch", "sim_gz.launch.py")
            ),
            launch_arguments={"slam": "mapping"}.items(),
        ),
    ])
