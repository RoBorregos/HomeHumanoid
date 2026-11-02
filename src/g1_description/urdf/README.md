# g1_description/urdf

Placeholder — pending M2a.

Steps to populate this directory (see `docs/plan.md`, Track A):

1. Clone `unitreerobotics/unitree_ros` (or download just `robots/g1_description/`).
2. Copy `g1_29dof.urdf` (and the meshes it references) here as `g1.urdf.xacro`,
   converting to xacro if parameterization is wanted (e.g. Dex3/Inspire hand variant).
3. Wrap it with the same patterns
   `vendor/home2/simulation/frida_gz_sim/urdf/` uses for FRIDA:
   - `<gazebo>` + the `gz_ros2_control` plugin pointing to a `controller_manager` config.
   - Gazebo's `VelocityControl` plugin on the base/pelvis link (see Track A
     in the plan: legs in an idle pose, the base moves kinematically via the
     `cmd_vel` coming from `g1_base_bridge`).
   - Sim sensors: `gpu_lidar` (analogous to FRIDA's dual RPLIDAR) simulating the
     Livox MID-360, and an RGB-D camera simulating the D435i.
4. Update `launch/description.launch.py` to point at the final xacro.

The real URDF isn't vendored in this initial scaffold because it requires
Unitree's binary meshes (STL/DAE), which are heavy — left as a manual step
for the team.
