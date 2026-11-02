# HomeHumanoid

Simulation and navigation for the **Unitree G1** humanoid, built as an extension of RoBorregos' RoboCup@Home stack (`home2`, currently running on **FRIDA**: omnidirectional base + xArm6).

See the full milestone plan in `docs/plan.md`.

## Why this repo exists

- Unitree doesn't ship an official navigation/Nav2 stack for the G1 — only simulation and low-level locomotion control (`unitree_mujoco`, `unitree_rl_lab`, `unitree_sdk2`). Nav2/SLAM has to be assembled on top.
- `home2` already has a mature Nav2 stack (ROS 2 Jazzy, MPPI, `slam_toolbox`, Gazebo Harmonic) that is mostly platform-agnostic. The FRIDA-specific parts (the `omnidriver` driver, URDF description) are the only pieces that need replacing.
- This repo keeps `home2` as a dependency (submodule) instead of forking it, and only adds the new packages the G1 needs.

## Structure

```
HomeHumanoid/
  vendor/home2/           # git submodule of RoBorregos/home2 (pinned to a commit)
  docker/                 # Dockerfile + compose (cpu/cuda/l4t) + run.sh
  src/
    g1_description/       # G1 URDF/xacro + simulation plugins
    g1_base_bridge/        # cmd_vel -> locomotion bridge (Gazebo VelocityControl | RL policy)
    g1_sim_bringup/         # launch files per simulation track
    g1_nav_overlay/          # Nav2/SLAM configs adapted to the G1 footprint
  third_party/            # unitree_mujoco, unitree_rl_lab (cloned, not submodules)
  worlds/                 # Gazebo worlds generated from maps
  docs/plan.md            # milestone plan
```

## Setup (to be completed per M0)

```bash
git submodule add https://github.com/RoBorregos/home2 vendor/home2
git submodule update --init --recursive
./docker/run.sh cpu   # or cuda / l4t on the Jetson Orin
```

Inside the container:

```bash
colcon build --symlink-install
```
