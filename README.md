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
  lib.sh                   # helpers shared by run.sh and docker/<area>/run.sh (as in home2)
  docker/navigation/       # navigation container: Dockerfile, compose, run.sh
  docker/simulation/       # simulation container (Gazebo + MuJoCo)
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

Or run everything in containers, one per area, see [Containers](#containers-one-per-area-like-home2) below. The team Orin runs it natively too.

## Step-by-step guide: using the G1

New to the repo? Follow this in order. Every command runs from the repo root (`cd ~/HomeHumanoid`). The reference table for `./run.sh` comes after this section.

### 0. Before anything (shared Orin)

Steps 0–4 are simulation, on the team Orin `frida-jazzy`; the real G1 runs on its own Orin (step 5). `frida-jazzy` also runs home2's containers. Give your session its own ROS and Gazebo network, so your simulated G1 doesn't mix topics with FRIDA's nodes:

```bash
export ROS_DOMAIN_ID=77          # any number not used by home2 (it uses the default 0)
export GZ_PARTITION=g1_$USER
```

Put both in every terminal you open, including the ones for `--check`, `--ui` or `ros2 topic`. A terminal without them won't see the robot. Don't reboot the Orin without asking the team.

### 1. First time: build

```bash
git submodule update --init --recursive
./src/g1_description/scripts/vendor_g1.sh     # G1 URDF + meshes (once)
./run.sh --build                               # ~5 min on the Orin
```

After a `git pull` that changes code, run `./run.sh --build` again. Python scripts and YAML files also need it, because ROS runs the installed copy.

### 2. First run: watch the G1 navigate in Gazebo

This is the simplest way to see the full stack working. It uses the arena map the G1 already made (`g1_robocup2026_1`).

```bash
./run.sh sim --headless -d        # Gazebo + localization + Nav2 + nav_central, in the background
./run.sh --status                 # what is running; logs in /tmp/g1nav
```

Wait about a minute: `nav_central` checks the sensors and TFs, then starts Nav2 (follow it with `tail -f /tmp/g1nav/sim_launch.log`). Then send the robot to places, through the same services home2's `task_manager` uses:

```bash
./run.sh --check --tour kitchen living_room exit
```

Places in this map: `start_location`, `start_area`, `inspection_point`, `exit`, `entrance`, `laundry`, `bedroom`, `living_room`, `kitchen`. Each goal should end in `SUCCEEDED`.

To watch it, start the sim without `--headless` from a desktop session (Gazebo window), or open `./run.sh --ui` (needs a display: VS Code/X forwarding or the Orin's screen) to see the map and click goals.

When done:

```bash
./run.sh --stop
```

### 3. Map a new place (sim)

```bash
./run.sh sim --mapping --headless -d --nav-ui
#   in nav_ui, send goals inside the part of the map already built; the map grows as it moves
./run.sh --save-map my_map        # -> src/g1_navigation/maps/my_map.*
./run.sh --stop
./run.sh --tagger                 # open maps/my_map.yaml, draw areas/places, save as maps/areas/areas_my_map.json
./run.sh sim --map my_map -d      # navigate on the new map
./run.sh --check --tour all
```

A map is only useful for the robot that made it: the G1 mislocalizes on FRIDA's map (`robocup2026_1`), so map each arena with the G1.

### 4. Real walking: MuJoCo

MuJoCo runs Unitree's RL walking policy, so the robot really walks on its legs. There's no lidar: it uses a saved map and its odometry.

```bash
./run.sh mujoco -d                       # starts MuJoCo, stands up, releases the band
./run.sh --check --tour kitchen exit
./run.sh --odom-error                    # how far leg odometry drifts vs the truth
./run.sh --stop
```

If it says `g1_ctrl is not in Velocity ... the robot would fall`, run `./run.sh --stop` and try again.

### 5. Real G1: on the humanoid's own Orin (not tested on hardware yet)

On the robot everything runs on the **Orin inside the G1** (Unitree's development computer, `192.168.123.164` on the robot's internal network), not on `frida-jazzy`. The lidar (`192.168.123.120`) and Unitree's locomotion service are on that network, and `./run.sh robot` defaults to it (`--iface eth0 --host-ip 192.168.123.164`). Check both on the robot with `ip -br addr`.

**One-time setup on the robot's Orin.** Check the OS with `lsb_release -a`:

- **Ubuntu 24.04 + ROS 2 Jazzy installed:** run natively.
  ```bash
  git clone -b Nav2_implementation https://github.com/RoBorregos/HomeHumanoid.git ~/HomeHumanoid
  cd ~/HomeHumanoid && git submodule update --init --recursive
  ./src/g1_description/scripts/vendor_g1.sh
  # unitree_sdk2 into /usr/local (g1_loco_bridge links it), see third_party/README.md
  git clone https://github.com/unitreerobotics/unitree_sdk2 third_party/unitree_sdk2
  cmake -S third_party/unitree_sdk2 -B third_party/unitree_sdk2/build_local -DBUILD_EXAMPLES=OFF
  sudo cmake --build third_party/unitree_sdk2/build_local --target install && sudo ldconfig
  ./third_party/fetch_lio.sh        # Livox driver + FAST-LIO (~15 min)
  ./run.sh --build
  ```
- **Any other OS (the G1 may ship with 20.04 / JetPack 5): containers.** Jazzy needs 24.04, which the images bring. Only Docker is needed on the robot.
  ```bash
  git clone https://github.com/RoBorregos/HomeHumanoid.git ~/HomeHumanoid
  cd ~/HomeHumanoid && git submodule update --init --recursive
  ./src/g1_description/scripts/vendor_g1.sh
  ./run.sh navigation --build --build-image --mode robot --nav    # first time: image + colcon build
  ```
  Then use `./run.sh navigation ... --mode robot` instead of `./run.sh robot` below (see [Containers](#containers-one-per-area-like-home2)). The navigation image does not use CUDA, so an older JetPack driver is not a problem for it.

`ROS_DOMAIN_ID` doesn't need changing there (home2 doesn't run on the robot), unless the robot's Orin also runs other ROS nodes.

**Before the first run:**
- G1 on the gantry, someone holding the remote's e-stop, until mapping and short goals work.
- Put the G1 in walking mode with the remote (or pass `--auto-start` so the bridge does it).
- First check the sensors with `--no-nav`: `/livox/lidar` and `/livox/imu` should publish, and in RViz `/scan` should show the walls but not the robot's own arms. Open items to verify are in `docs/plan.md` (lidar orientation, body crop, footprint).

```bash
./run.sh robot --no-nav                        # sensors + odometry only
./run.sh robot --mapping                       # walk it around with goals (--nav-ui) to build the map
./run.sh --save-map lab
./run.sh --stop
./run.sh --tagger                              # save as maps/areas/areas_lab.json
./run.sh robot --map lab                       # navigate
```

Maps saved on the robot stay in its `src/g1_navigation/maps`: commit them from there, or copy them back. To watch from a laptop (RViz, `nav_ui`), connect it to the robot's network with the same `ROS_DOMAIN_ID`.

### 6. Using it from home2

With any target running (`sim`, `mujoco` or `robot`) on the same `ROS_DOMAIN_ID`, home2's `task_manager` (for example `nav_tasks.py` in a GPSR task) drives the G1 like FRIDA: `/navigation/go_to_map_area`, `/navigation/get_robot_pose`, patrols and so on. Nothing in home2 changes.

### When something fails

| Symptom | What to do |
|---|---|
| `Something is already running, stop it first` | `./run.sh --stop`. If something remains, `./run.sh --status` shows its PID |
| `Not built yet` | `./run.sh --build` |
| `--check` hangs or sees no services | The terminal lacks the same `ROS_DOMAIN_ID`, or the stack is still starting: look at `/tmp/g1nav/*.log` |
| Goals fail right at the start | Wait for `nav_central` to finish, or the robot is lost: set the initial pose in `--ui` |
| `livox_ros_driver2 not found` | Real robot only: build `./third_party/fetch_lio.sh`, then `./run.sh --build` |
| Gazebo very slow | Another simulation is running on the Orin: `./run.sh --status`, `nvidia-smi`/`top` |

## Containers: one per area, like home2

`./run.sh <area>` runs each area in its own container, with home2's layout: `docker/<area>/{Dockerfile, docker-compose.yaml, run.sh}`, the repo mounted at `/workspace/src`, and `build/ install/ log/` per area under `docker/<area>/`. The idea is the same as in home2: every area is a self-contained unit that can move to the robot's Orin as it is.

| Area | Image | What runs inside | Where |
|---|---|---|---|
| `navigation` | on `roborregos/home2:<env>_base` | lidar pipeline, FAST-LIO, slam_toolbox, Nav2, `nav_central`, `semantic_nav`; with `--mode robot` also the MID-360 driver and `g1_loco_bridge` (what FRIDA's `omnidriver` is in home2's navigation container) | dev machines and the G1 |
| `simulation` | on `ros:jazzy` | Gazebo (world, G1, ros_gz bridge, odometry) and MuJoCo (`unitree_mujoco`, `g1_ctrl`, `g1_rl_bridge`) | dev machines only |
| `vision`, `hri`, `manipulation`, ... | home2's | forwarded to `vendor/home2/run.sh` | as in home2 |

The navigation image is built on home2's base image on purpose: on the G1's Orin it shares its layers (ROS, CycloneDDS, and on l4t CUDA/OpenCV/torch) with home2's vision and hri images, which are to move there too. All containers use the host network and the same `ROS_DOMAIN_ID` (taken from your shell), so they talk to each other and to home2's.

```bash
# first time (image + colcon build; images take a while)
./run.sh simulation --build --build-image
./run.sh navigation --build --build-image

# Gazebo + navigation, then the same checks as natively
./run.sh simulation --gz -d                       # Gazebo, the G1, its sensors and odometry
./run.sh navigation --nav -d                      # lidar pipeline, localization, Nav2, nav_central
./run.sh navigation --check --tour kitchen exit   # tools run in the running container
./run.sh navigation --stop && ./run.sh simulation --stop

# mapping in Gazebo
./run.sh simulation --gz -d
./run.sh navigation --mapping --nav-ui -d
./run.sh navigation --save-map my_map
./run.sh navigation --tagger

# MuJoCo
./run.sh simulation --mujoco -d
./run.sh navigation --nav --mode mujoco

# real G1, on its Orin
./run.sh navigation --mapping --mode robot
./run.sh navigation --nav --mode robot --map lab
```

`./run.sh navigation` or `./run.sh simulation` with no task opens a shell in the container. `--stop`, `--down`, `--recreate`, `--clean` and `-d` work as in home2. `G1_MODE=robot` in the robot's shell makes `--mode robot` the default there.

The native targets below (`./run.sh sim|mujoco|robot`) still work and run the same launch files; the containers only split them: `sim_gz.launch.py nav:=false perception:=false` in `simulation`, `g1_bringup/nav_area.launch.py` in `navigation`.

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
./run.sh sim --mapping --headless   # SLAM + Nav2 on the growing map; drive it with goals (--nav-ui),
                                    #   each inside the map built so far
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
