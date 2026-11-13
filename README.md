# HomeHumanoid

Navigation and simulation for the **Unitree G1** humanoid, on top of RoBorregos' RoboCup@Home stack ([home2](https://github.com/RoBorregos/home2), the one FRIDA runs).

The G1 offers the **same `/navigation/*` services as FRIDA**, so home2's `task_manager` drives it without changes. The G1 maps with the Livox MID-360 in its head: the 3D cloud is flattened to a 2D scan for slam_toolbox (maps in home2's format) and also feeds the local costmap in 3D.

## Containers 

| Area | What runs inside | Where |
|---|---|---|
| `simulation` | Gazebo (arena + G1 + simulated lidar) and MuJoCo (G1 walking with Unitree's RL policy) | your computer |
| `navigation` | lidar filtering, SLAM, Nav2, `nav_central`; on the robot also the lidar driver and locomotion | your computer and the G1 |
| `vision`, `hri`, ... | forwarded to home2's `./run.sh` | as in home2 |

## Quick start

Needs Docker. From the repo root:

```bash
git submodule update --init --recursive
./src/g1_description/scripts/vendor_g1.sh          # G1 model (once)

./run.sh simulation --build --build-image          # first time: image + build (slow)
./run.sh navigation --build --build-image

./run.sh simulation --gz -d                        # Gazebo with the G1
./run.sh navigation --nav -d                       # localization + Nav2 + nav_central
./run.sh navigation --check --tour kitchen exit    # send it to places
./run.sh navigation --stop && ./run.sh simulation --stop
```

On a shared computer (the team Orin), first `export ROS_DOMAIN_ID=77` in every terminal so your G1 doesn't mix with other robots.

Places in the default map (`g1_robocup2026_1`): `kitchen`, `living_room`, `bedroom`, `laundry`, `entrance`, `exit`, `inspection_point`, `start_area`, `start_location`.

## Map a new place

```bash
./run.sh simulation --gz -d
./run.sh navigation --mapping --nav-ui -d          # send goals in nav_ui; the map grows
./run.sh navigation --save-map my_map              # -> src/g1_navigation/maps/
./run.sh navigation --tagger                       # draw the areas -> maps/areas/areas_my_map.json
./run.sh navigation --nav --map my_map
```

## MuJoCo 

```bash
./run.sh simulation --mujoco -d                    # stands up and releases the band
./run.sh navigation --nav --mode mujoco
```

## Real G1 

Runs on the G1's own Orin. Keep it on the gantry with the remote's e-stop in hand, and put it in walking mode with the remote.

```bash
./run.sh navigation --mapping --mode robot
./run.sh navigation --save-map lab
./run.sh navigation --nav --mode robot --map lab
```

## Help

- `./run.sh simulation --help`, `./run.sh navigation --help`: all tasks and flags.
- [docs/usage.md](docs/usage.md): running without Docker, every flag, real-robot checklist, troubleshooting.
- [docs/plan.md](docs/plan.md): design, milestones and test results.

## Structure

```
docker/<area>/     Dockerfile, compose and run.sh per area
src/               G1 packages: description, perception, navigation, bridges, bringup
vendor/home2/      home2 (submodule): interfaces, Gazebo worlds, the other areas
third_party/       Unitree and Livox code (cloned, not in git)
```
