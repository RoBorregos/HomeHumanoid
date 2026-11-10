# HomeHumanoid

Simulation and navigation for the **Unitree G1** humanoid, built as an extension of RoBorregos' RoboCup@Home stack (`home2`, currently running on **FRIDA**: omnidirectional base + xArm6).

The G1 exposes the **same `/navigation/*` interface as FRIDA** (`nav_central`, `semantic_nav`, areas, patrols), so home2's `task_manager` drives it unchanged. See the full milestone plan and results in `docs/plan.md`.

## Why this repo exists

- Unitree doesn't ship an official navigation/Nav2 stack for the G1 — only simulation and low-level locomotion control (`unitree_mujoco`, `unitree_rl_lab`, `unitree_sdk2`). Nav2/SLAM has to be assembled on top.
- `home2` already has a mature Nav2 stack (ROS 2 Jazzy, slam_toolbox, Nav2, `nav_central`, area tagging) that is mostly platform-agnostic. Its FRIDA-specific parts (two 2D lidars, `omnidriver`, the xArm frames) are what this repo replaces.
- `home2` stays a dependency (submodule, for `frida_interfaces`/`frida_constants` and the Gazebo worlds); the navigation nodes are ported into `g1_navigation` and adapted.

## Structure

```
HomeHumanoid/
  vendor/home2/            # git submodule of RoBorregos/home2 (pinned to a commit)
  docker/                  # Dockerfile + compose (cpu/cuda/l4t) + run.sh
  src/
    g1_description/        # G1 URDF/xacro, 3D MID-360 + IMU sim sensors, base_link -> pelvis
    g1_perception/         # MID-360: body crop, /scan projection, FAST-LIO odometry adapter
    g1_navigation/         # home2's nav_main + map_context ported: nav_central, semantic_nav,
                           #   person_goal_smoother, table_docker, nav_ui, tagger, launch, maps/
    g1_nav_overlay/        # Nav2 / slam_toolbox configs per mode (gz, mujoco, robot)
    g1_base_bridge/        # Track A: cmd_vel -> Gazebo VelocityControl, dead-reckoned odom
    g1_rl_bridge/          # Track B: cmd_vel -> unitree_rl_lab policy in MuJoCo (+ leg odom)
    g1_loco_bridge/        # Real robot: cmd_vel -> LocoClient::SetVelocity (+ leg odom)
    g1_leg_odom/           # Leg-inertial odometry estimator (header-only + node)
    g1_sim_bringup/        # sim_gz / sim_mapping / sim_mujoco launch files
    g1_bringup/            # robot.launch.py for the real G1 (untested on hardware)
  third_party/             # unitree_mujoco, unitree_rl_lab, unitree_sdk2, lio_ws (cloned)
  docs/plan.md             # milestone plan and findings
```

## How mapping works with the 3D lidar

The G1 has one Livox MID-360 in its head instead of FRIDA's two 2D lidars:

```
MID-360 ──> /livox/lidar ──> body crop ──> /livox/points_filtered ──> VoxelLayer (local costmap)
   │                                              └──> pointcloud_to_laserscan (0.10–1.50 m) ──> /scan
   └──> /livox/imu ──> FAST-LIO ──> odom -> base_link (robot; bridges' odometry in sim)
/scan ──> slam_toolbox (mapping / localization) ──> map -> odom, /map (.pgm/.yaml/.posegraph)
```

The 3D cloud is flattened to a 2D scan that sees obstacles at any height between 0.10 and 1.50 m, so the maps stay in home2's format and its tools (`map_area_tagger`, `areas.json`, `nav_ui`, `semantic_nav`) keep working. The frame tree is `map -> odom -> base_link -> pelvis -> ...`, with `base_link` the footprint on the floor.

## Setup

```bash
git submodule update --init --recursive
./src/g1_description/scripts/vendor_g1.sh        # G1 URDF + meshes (gitignored)
./third_party/fetch_lio.sh                       # Livox-SDK2, livox_ros_driver2, FAST-LIO (robot / LIO)
source /opt/ros/jazzy/setup.bash
source third_party/lio_ws/install/setup.bash     # if built
colcon build
```

Or `./docker/run.sh cpu|cuda|l4t` and build inside the container (`./run.sh --build` does the `colcon build`). The Orin runs it natively.

## Running it: `./run.sh`

One entry point, like home2's `./run.sh`: a target (where the G1 runs), a task, and flags. `./run.sh --help` lists everything.

```bash
./run.sh --build                    # colcon build (optionally: --build g1_rl_bridge ...)
```

| Target | What it is | Tasks |
|---|---|---|
| `sim` | Gazebo: kinematic base, simulated MID-360 | `--mapping`, `--nav` |
| `mujoco` | MuJoCo: the RL policy walking on its legs, static map, no lidar | `--nav` |
| `robot` | The real G1. **Untested on hardware** | `--mapping`, `--nav` |

`--nav` is the default task: localization on a saved map + Nav2 + `nav_central`.

**Map → tag → navigate** (same workflow as home2):

```bash
./run.sh sim --mapping --headless   # SLAM + Nav2 on the growing map; drive it with goals (--nav-ui)
./run.sh --save-map my_map          # -> src/g1_navigation/maps/my_map.{pgm,yaml,posegraph,data}
./run.sh --stop
./run.sh --tagger                   # -> maps/areas/areas_my_map.json
./run.sh sim --map my_map           # localization + Nav2 + nav_central
./run.sh --check --tour all         # the services home2's task_manager uses
```

**MuJoCo** (starts `unitree_mujoco`, `g1_ctrl` and the bridge, then lowers and releases the elastic band; without a display it uses Xvfb):

```bash
./run.sh mujoco -d                  # -d: return once it is up
./run.sh --odom-error               # leg odometry drift against MuJoCo's ground truth
./run.sh --check --tour kitchen exit
./run.sh --stop
```

**Real G1** (gantry and the remote's e-stop first; put it in walking mode with the remote):

```bash
./run.sh robot --mapping --iface eth0 --host-ip 192.168.123.164
./run.sh --save-map my_map
./run.sh robot --map my_map
```

| Flag | Applies to | Meaning |
|---|---|---|
| `--map NAME` | all | Map to localize on. Default `$MAP_NAME`, else `g1_robocup2026_1` |
| `--no-nav` | all | Robot or simulator only, no navigation stack |
| `--nav-ui` | sim, robot | Also open `nav_ui` |
| `--odom SRC` | all | sim: `bridge` \| `lio`. mujoco: `leg` \| `ground_truth`. robot: `lio` \| `leg` |
| `--headless` | sim | No Gazebo window (default without `DISPLAY`) |
| `--display :N` | mujoco | X display for the MuJoCo window |
| `--keep-band` | mujoco | Leave the robot on the elastic band (`./run.sh --release-band` later) |
| `--iface`, `--host-ip` | robot | Network interface and this computer's address on the G1 network |
| `--auto-start` | robot | Let the bridge put the G1 in walking mode itself |
| `-d` | all | Detached. Logs in `/tmp/g1nav` (`G1_LOG_DIR`) |
| `key:=value` | all | Passed through to the launch file |

`./run.sh --status` shows what is running and `./run.sh --stop` stops what it started, with SIGINT. Without `-d` it stays in the foreground and Ctrl-C stops everything. The launch files (`g1_sim_bringup`, `g1_bringup`, `g1_navigation`) can still be used directly; `src/g1_navigation/scripts/run_nav.sh` holds the navigation-only commands that `run.sh` forwards to.

`g1_robocup2026_1` is the arena mapped by the G1's own simulated MID-360; `robocup2026_1` is FRIDA's map, kept for reference (the G1 mislocalizes on it, see `docs/plan.md`).
