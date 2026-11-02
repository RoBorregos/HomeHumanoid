# Plan: Navigation simulation for the Unitree G1 (RoBorregos home2 → humanoid)

## Context

RoBorregos wants to start porting its RoboCup@Home stack (repo `home2`, currently built for **FRIDA**: omnidirectional base + xArm6 arm) toward the **Unitree G1** humanoid, which the team already physically owns. Before touching the real robot, the goal is to validate **navigation in simulation**.

Research confirmed two key facts that shape this plan:
- **Unitree does not offer an official Nav2/navigation stack for the G1.** Its repos (`unitree_mujoco`, `unitree_rl_lab`, `unitree_sim_isaaclab`, `unitree_ros2`, `unitree_sdk2`) provide simulation and low-level locomotion control (via DDS/`unitree_hg`), but SLAM/Nav2 has to be assembled on top, the way community repos do (`Skvayzer/humanoid_navigation`, etc.).
- **home2 already has a mature, reusable Nav2 stack** (ROS 2 Jazzy, Nav2 + MPPI configured for a holonomic base, `slam_toolbox`, Gazebo Harmonic, Docker with `run.sh` auto-detecting cpu/cuda/l4t). The only truly platform-specific parts are `omnidriver` (the base's ODrive driver) and FRIDA's URDF description/sim plugins — everything else (Nav2 configs, `task_manager` orchestration, vision, HRI, `frida_interfaces`) is reasonably robot-agnostic.

Decisions already made with the user:
- Hardware: no GPU on the laptop, but **a Jetson Orin with a GPU is available** for the heavy simulation/RL workloads.
- The team already has the **physical G1**, so a sim-to-real phase is within this cycle's horizon (not purely speculative).
- Repo strategy: a **new, separate `HomeHumanoid` repo** that reuses pieces of `home2` (no fork, no folder inside home2).

The goal of this plan is to leave scaffolding + a milestone path so the team can incrementally get the G1 walking in simulation and then responding to Nav2 goals the same way FRIDA does today.

## Repo strategy

- `HomeHumanoid` is a new, independent **ROS 2 Jazzy workspace**.
- `home2` is added as a **git submodule** at `vendor/home2`, pinned to a known commit — so platform-agnostic packages (`frida_interfaces`, `task_manager`, `vision`, `hri`) build directly with `colcon build` by ROS package name, without copy-pasting code that later drifts. The submodule is bumped deliberately, not automatically.
- G1-specific packages live natively in `HomeHumanoid/src/` and depend on home2's packages by ROS name, not by vendoring source.
- Revisit later: if the G1 work matures, it can be promoted into a `platforms/g1/` folder inside `home2` itself — that decision is deferred until after M3.

## Simulator: two tracks, not one

home2's Nav2 assets are "Gazebo-flavored"; Unitree's locomotion assets are "MuJoCo/Isaac-flavored." Instead of picking one, run both tracks:

- **Track A — Gazebo Harmonic (legs in a static/idle pose):** import the G1's URDF (canonical source: `unitreerobotics/unitree_ros` → `robots/g1_description/g1_29dof.urdf`, since `unitree_model` is deprecated) wrapped the same way `frida_gz_sim/urdf` wraps FRIDA's xacro, with `gz_ros2_control` + Gazebo's `VelocityControl` plugin acting on the base link — exactly how FRIDA's holonomic chassis moves in sim today. This decouples "does Nav2/costmaps/SLAM/BTs work with the G1's footprint?" from "can the G1 actually walk?", and lets `nav2_sim_overlay.yaml`, `mapper_params_sim.yaml`, and `world_from_map.py` be reused almost unchanged.
- **Track B — `unitree_mujoco` + `unitree_rl_lab` (real locomotion):** once Track A validates the nav stack, swap the locomotion backend for the real G1-29dof RL walking policy running in MuJoCo (same DDS protocol as the real robot), driven by the same `cmd_vel` Nav2 produces. This is the real sim-to-real rehearsal.
- `unitree_sim_isaaclab` (Isaac Lab) is deferred to the Jetson Orin for later stages that need richer sensors (simulated RGB-D/LiDAR) or manipulation — not needed for the navigation MVP.

## Locomotion bridge as a "virtual base"

A single node, `g1_base_bridge`, subscribes to the `cmd_vel` Nav2 produces (respecting home2's `stamped_cmd_vel` convention) and republishes it depending on the active backend:
- Track A: straight to the command topic of Gazebo's `VelocityControl` plugin (same as FRIDA). Pure `rclpy`, no DDS/Unitree SDK involved.
- Track B: **confirmed on the Orin (see "M1 findings" below) to NOT be a `/command` string topic** — the original guess was wrong. The real mechanism is detailed below; `g1_base_bridge`'s Track B variant must be a hybrid process that is both an `rclpy` node (to receive Nav2's `cmd_vel`) and a `unitree_sdk2_python` DDS participant (to relay/override `rt/lowstate`), per the "M1 findings" section.

This is the only new, G1-specific piece of glue code that needs writing; everything else in Nav2/SLAM is parameter reuse.

## M1 findings — G1 locomotion control protocol (confirmed on the Orin, resolves open questions #1 and #3)

M1 was run on the team's Jetson Orin (HRI Orin, ssh over Tailscale). Key corrections to the assumptions above, verified by reading `unitree_sdk2`/`unitree_rl_lab` source directly (not guessed):

- **There is no separate `WirelessController_`/`rt/wirelesscontroller` topic for G1, and no `/command` string topic.** The joystick/remote-control state is a raw 40-byte blob (`wireless_remote`, type `REMOTE_DATA_RX`) **embedded inside the `unitree_hg::msg::dds_::LowState_` message itself**, on the same topic (`rt/lowstate`) the simulator (or real robot) already publishes continuously for motor/IMU telemetry. `unitree_sdk2`'s `unitree_joystick.hpp` defines the exact byte layout:
  - bytes 0-1: header (unused by the software layer).
  - bytes 2-3: a 16-bit button bitmask (`R1`=bit0, `L1`=bit1, `Start`=bit2, `Select`=bit3, `R2`=bit4, `L2`=bit5, `f1`=bit6, `f2`=bit7, `A`=bit8, `B`=bit9, `X`=bit10, `Y`=bit11, `up`=bit12, `right`=bit13, `down`=bit14, `left`=bit15).
  - bytes 4-7: `lx` (float), 8-11: `rx`, 12-15: `ry`, 16-19: `L2` (analog, unused by `extract()`), 20-23: `ly`. Bytes 24-39 are padding.
  - `unitree_rl_lab`'s `UnitreeJoystick::extract()` reads `up/down/left/right` and `LT`/`RT` directly from these **bits** (not separate analog axes) — despite physical-gamepad docs calling LT/RT "triggers," in this software layer they only need to be 0 or 1.
- **The G1's own FSM (in `unitree_rl_lab/deploy`) gates locomotion through states `Passive -> FixStand -> Velocity`**, transitioning on edge-triggered button combos read from `lowstate->joystick` (e.g. `"LT + up.on_pressed"`, `"RB + X.on_pressed"`, defined in `deploy/robots/g1_29dof/config/config.yaml`). Once in `Velocity`, the RL walking policy reads `lx`/`ly`/`rx` as forward/strafe/yaw velocity commands. **There's no lower-level entry point that bypasses this FSM** — any caller (a real gamepad, a Nav2 bridge, or a test script) has to drive it through these same joystick fields.
- **No physical gamepad was available, and this Jetson's kernel has no `uinput` support at all** (`modprobe uinput` fails — not built as a module and `/dev/uinput` doesn't exist), so creating a virtual joystick device was not possible. **Publishing a second, standalone `LowState_` message on `rt/lowstate` to fake the joystick is unsafe**: it would race with the simulator's own continuous publish and corrupt the motor/IMU fields `g1_ctrl`'s control loop actually depends on.
- **The pattern that works, and is the one `g1_base_bridge`'s Track B should use in production**: a small relay process subscribes to the real `rt/lowstate`, and on every message, republishes an identical copy **except** with the `wireless_remote` bytes overridden to the desired button/axis state. Because the relay re-publishes immediately after receiving the real sample, its copy is always the freshest one on the topic, so the FSM sees correct motor/IMU telemetry at all times plus the injected joystick state. This was validated on the Orin: a throwaway C++ relay (`third_party/lowstate_relay.cc`, built against the already-installed `unitree_sdk2`) drove `Passive -> FixStand -> Velocity` successfully — confirmed in `g1_ctrl`'s own log:
  ```
  FSM: Change state from Passive to FixStand
  FSM: Change state from FixStand to Velocity
  ```
  A second relay (`third_party/walk_relay.cc`) then set `lx=0.3` for 4s to command forward walking (not yet visually confirmed on this machine — the Orin's RDP session had a display issue after an unrelated reboot — but the control path is proven end-to-end).
- **Resolves open question #3 (ROS 2 vs. raw DDS boundary)**: the clean boundary is NOT "ROS 2 only touches `g1_base_bridge` from outside." Track B's `g1_base_bridge` must itself be a **hybrid process**: an `rclpy` node (subscribing to Nav2's `cmd_vel`) that also opens a `unitree_sdk2_python` DDS participant and runs the lowstate-relay-with-override pattern above, translating `Twist.linear.x/y` and `Twist.angular.z` into the `lx`/`ly`/`rx` fields each time a new `rt/lowstate` sample arrives. This needs `unitree_sdk2_python` installed (pip, needs CycloneDDS headers) as a new dependency of the `g1_base_bridge` package — not previously listed.
- **M1's deliverable (G1 FSM reaching `Velocity`, i.e. "standing and ready to walk") is effectively done**, modulo a visual recording once the Orin's display issue is sorted — the underlying DDS control path is proven via `g1_ctrl`'s own log output, independent of the viewer.

## SLAM / localization

Use `slam_toolbox` (matches home2's current default, which is itself moving away from RTAB-Map) fed by a 2D scan generated from the G1's Livox MID-360 point cloud via `pointcloud_to_laserscan`. The forward-facing D435i remains available for vision-based costmap layers later. 3D SLAM approaches like Fast-LIO/NDT (as used by community G1 nav repos) are deferred to an optional later milestone — not required to prove Nav2 goal-following in sim.

## Milestones

- **M0 — Workspace scaffold.** Create the ROS 2 Jazzy workspace, Docker mirroring home2's `run.sh`/`Dockerfile.ROS` pattern (cpu/cuda variants; l4t for the Orin later), add home2 as a submodule, confirm `colcon build` of the vendored packages. Deliverable: buildable workspace + CI that builds the submodule.
- **M1 — G1 standing/teleop in MuJoCo (no ROS 2 yet). Done (see "M1 findings" above), pending visual recording.** Built `unitree_mujoco` + `unitree_rl_lab`'s `g1_ctrl` on the Orin against a locally-installed `unitree_sdk2` (two installs were needed: one at `/opt/unitree_robotics` per `unitree_mujoco`'s README, one at the default `/usr/local` prefix per `unitree_rl_lab`'s README, since their CMakeLists hardcode different paths). No physical gamepad was available and this Jetson kernel has no `uinput`, so FSM transitions were driven via the `rt/lowstate` relay-with-override pattern instead — confirmed working via `g1_ctrl`'s log (`FSM: Change state ... to Velocity`). Deliverable: recording of the G1 walking under the RL policy in MuJoCo — still pending, blocked on an unrelated Orin RDP/display issue after a reboot.
- **M2a — Nav2 goal-following over a kinematic virtual base (Track A, Gazebo).** Create `g1_description` (wrapped URDF + sim plugins), adapt `nav2_standard.yaml`/`omni_config` to the G1's footprint, write `g1_base_bridge` (passthrough-to-`VelocityControl` variant). Reuse `world_from_map.py` with an existing home2 map for a quick first test. Deliverable: a Nav2 goal makes the G1 (idle legs) glide to the destination in Gazebo using MPPI + slam_toolbox, just like FRIDA does today.
- **M2b — Real locomotion (Track B, MuJoCo + RL policy).** Extend `g1_base_bridge` to target `unitree_rl_lab`'s command interface instead of `VelocityControl`; wire odometry from `SportModeState` into TF/`nav_msgs/Odometry`. Deliverable: the same goal-following demo, but the G1 actually walks via the RL policy in MuJoCo, driven end-to-end by Nav2.
- **M3 — SLAM-built map + autonomous exploration.** Simulate the Livox MID-360 (via `pointcloud_to_laserscan`) and/or the D435i on whichever backend proved more stable in M2; run `slam_toolbox` in mapping mode, save a map, rerun Nav2 against it. Deliverable: the G1 autonomously maps and navigates a simulated scenario from scratch.
- **M4 — Integration with home2's `task_manager`/vision/HRI.** Wire `task_manager`'s orchestration (first confirm its internal mechanism: BT, state machine, or LLM planner) to emit nav goals to the G1 stack instead of FRIDA's; bring in vision (YOLO/VLM) and HRI (STT/NLP) unmodified, confirming they're robot-agnostic. Deliverable: a GPSR-style demo ("go to X, find Y") running on the G1 sim stack.
- **M5 — Sim-to-real toward the physical G1.** Since the hardware is already owned, this phase is in scope for this cycle: replace MuJoCo with real `unitree_sdk2`/DDS on the robot, validate the same `g1_base_bridge` and Nav2 configs against real sensor data and real locomotion latency/safety margins.

## Directory structure (HomeHumanoid)

```
HomeHumanoid/
  vendor/home2/                  # git submodule, pinned commit
  docker/
    Dockerfile.ROS
    compose/{cpu.yaml,cuda.yaml,l4t.yaml}   # l4t for the Orin
    run.sh                       # mirrors home2's auto-detect pattern
  src/
    g1_description/              # URDF/xacro wrapping unitree_ros g1_description + sim plugins
    g1_base_bridge/               # cmd_vel -> VelocityControl (Track A) | unitree_rl_lab command (Track B)
    g1_sim_bringup/               # launch files per track (sim_gz.launch.py, sim_mujoco.launch.py)
    g1_nav_overlay/               # nav2_standard overlay + omni_config adapted to the G1 footprint
  third_party/
    unitree_mujoco/              # cloned directly (fast-moving repo, not a submodule)
    unitree_rl_lab/
  worlds/                        # reused output of world_from_map.py
  README.md
```

## Open questions to resolve when implementation starts (do not block the scaffold)

1. ~~`unitree_rl_lab`'s command interface~~ — **Resolved**, see "M1 findings" above: it's the `wireless_remote` bytes inside `rt/lowstate`, not a `/command` topic.
2. **`task_manager`'s internal mechanism** in home2 (BT vs. state machine vs. LLM planner) — confirm before M4, since it affects how nav goals get emitted to the new G1 stack.
3. ~~ROS 2 vs. raw DDS boundary~~ — **Resolved**, see "M1 findings" above: `g1_base_bridge` (Track B) is a hybrid `rclpy` + `unitree_sdk2_python` process, not a pure ROS 2 node.
4. **Walking not yet visually confirmed**: the forward-velocity relay (`walk_relay.cc`) ran without errors and the FSM reached `Velocity`, but actual walking motion hasn't been visually confirmed yet (Orin RDP display issue after a reboot). Re-confirm once the display is working, or by decoding joint motor positions from `rt/lowstate` over time as a non-visual check.
5. **G1 odometry source for Nav2's TF/`nav_msgs/Odometry`**: the plan originally assumed `SportModeState` would carry position/velocity (true for go2). **Confirmed false for G1**: `unitree_hg::msg::dds_::SportModeState_` only has `fsm_id`/`fsm_mode`/`task_id`/`task_time` — no pose. The real odometry source (state estimator fused from IMU + leg kinematics, possibly published elsewhere, or something `g1_base_bridge` has to compute itself) needs to be identified before M2b's odometry wiring can be written.

## Verification

- M0: `colcon build` with no errors over the full workspace (including the home2 submodule) inside the Docker container (cpu variant).
- M1: screen/video recording of the G1 standing and walking in MuJoCo via teleop and via the RL policy.
- M2a/M2b: `ros2 action send_goal /navigate_to_pose ...` moves the G1 (idle or walking) to the destination in Gazebo/MuJoCo respectively; verify in RViz that costmaps, TF, and the MPPI-planned trajectory look the same as FRIDA's existing demo.
- M3: run SLAM online, save a map, restart Nav2 against the saved map, confirm correct localization (AMCL/slam_toolbox localization mode).
- M4: run a sample GPSR-style task end-to-end and confirm vision/HRI/task_manager required no code changes, only robot configuration.
- M5: physical safety checklist (clear area, accessible emergency stop, conservative velocity limits) before any test on the real G1.
