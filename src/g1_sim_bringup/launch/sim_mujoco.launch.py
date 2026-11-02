"""Track B: Nav2 driving the G1's RL walking policy in unitree_mujoco.

M2b scaffold. Unlike sim_gz.launch.py, the simulator (unitree_mujoco) and the
RL policy controller (unitree_rl_lab's g1_ctrl) are NOT ROS 2 nodes — they are
external processes started by hand (or from a separate script) before running
this launch file. Config is via simulate/config.yaml, not CLI flags — see
third_party/README.md:

    cd third_party/unitree_mujoco/simulate/build && ./unitree_mujoco
    cd third_party/unitree_rl_lab/deploy/robots/g1_29dof/build && ./g1_ctrl --network lo

This launch file only brings up the ROS 2 side: the g1_base_bridge node in
rl_policy mode (see docs/plan.md, "M1 findings", for the confirmed
rt/lowstate wireless_remote mechanism it uses) and Nav2. Odometry is NOT yet
wired — G1's SportModeState_ (unitree_hg) only carries fsm_id/fsm_mode/
task_id/task_time, no position/velocity, unlike go2's; the real odometry
source for TF/nav_msgs/Odometry still needs to be identified (likely IMU
orientation + leg kinematics/state estimator — open question, see
docs/plan.md).
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    nav2_params_arg = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=PathJoinSubstitution(
            [
                get_package_share_directory("g1_nav_overlay"),
                "config",
                "nav2_standard.yaml",
            ]
        ),
        description="Nav2 overlay adapted to the G1's footprint",
    )

    base_bridge = Node(
        package="g1_base_bridge",
        executable="g1_base_bridge",
        name="g1_base_bridge",
        output="screen",
        parameters=[{"backend": "rl_policy"}],
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [get_package_share_directory("nav2_bringup"), "launch", "navigation_launch.py"]
            )
        ),
        launch_arguments={"params_file": LaunchConfiguration("nav2_params_file")}.items(),
    )

    return LaunchDescription([nav2_params_arg, base_bridge, nav2_bringup])
