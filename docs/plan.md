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
- **Resolves open question #3 (ROS 2 vs. raw DDS boundary)**: the clean boundary is NOT "ROS 2 only touches `g1_base_bridge` from outside." Track B's `g1_base_bridge` must itself be a **hybrid process**: an `rclpy` node (subscribing to Nav2's `cmd_vel`) that also opens a `unitree_sdk2_python` DDS participant and runs the lowstate-relay-with-override pattern above, translating `Twist.linear.x/y` and `Twist.angular.z` into the `lx`/`ly`/`rx` fields each time a new `rt/lowstate` sample arrives. This needs `unitree_sdk2_python` installed (pip, needs CycloneDDS headers) as a new dependency of the `g1_base_bridge` package — not previously listed.
- **M1 is fully confirmed, including visual proof of real walking.** The Orin's RDP session had a display issue after an unrelated reboot (couldn't connect to the real desktop), so `unitree_mujoco` was instead run under a headless `Xvfb` virtual display (`sudo apt install xvfb imagemagick xdotool`) and screenshotted directly (`DISPLAY=:99 import -window root shot.png`) — this is a reusable pattern for any future headless verification on this Orin. Two consecutive screenshots, ~2s apart, show the G1 in two completely different dynamic stride poses (one leg planted/one reaching forward, then a crouched running-like pose moments later) — conclusive evidence of continuous, active walking under the RL policy, not a frozen/static pose.
  - **Gotcha found along the way**: `State_RLBase`'s `enter()`/`run()` registers a per-tick `bad_orientation()` safety check that transitions straight back to `Passive` if tripped. The FSM bounced `Velocity -> Passive` within ~1ms on every attempt made *after* the G1 had been manually walked and had fallen once and the sim state was fixed via MuJoCo's "Reset" button — the elastic band's enable/disable runtime toggle is apparently independent of "Reset" (which only resets pose/velocity), so after a fall + manual band-release via key `9`, later `FixStand -> Velocity` attempts had no band support and tripped the orientation check. **Fix: fully restart `unitree_mujoco`** (not just click Reset) to get the band back to its config-default engaged state before retrying the FSM sequence.
  - The elastic band itself is controlled by **raw keyboard input to the MuJoCo window** (keys `7`/`8`/`9` to lower/lift/toggle), a completely separate channel from the `rt/lowstate`-embedded joystick bits used for the FSM — simulated here via `xdotool key --window <id> 8` against the Xvfb display. Real hardware won't have this band at all, so this is sim-only scaffolding, not something `g1_base_bridge` needs to replicate.
  - Releasing the band too early (before the policy has had a few seconds to settle into a stable stance after lowering) caused one fall — worth a deliberate settle delay in any future scripted sequence, though this doesn't block M2 since the band isn't part of the real bridge design.

## SLAM / localization

Implemented in M3 (see "M3/M4" below): `slam_toolbox` 2D, fed by `/scan` projected from the MID-360's 3D cloud between 0.10 and 1.50 m over the floor, so the maps stay in home2's format. A `VoxelLayer` on the cloud covers obstacles that a 2D map misses. FAST-LIO (MID-360 cloud + its IMU) gives odometry on the real robot, where the leg odometry drifts in yaw; in sim the bridges' odometry is exact and FAST-LIO is optional. The D435i remains available for vision-based costmap layers later.

## Milestones

- **M0 — Workspace scaffold.** Create the ROS 2 Jazzy workspace, Docker mirroring home2's `run.sh`/`Dockerfile.ROS` pattern (cpu/cuda variants; l4t for the Orin later), add home2 as a submodule, confirm `colcon build` of the vendored packages. Deliverable: buildable workspace + CI that builds the submodule.
- **M1 — G1 standing/teleop in MuJoCo (no ROS 2 yet). Done, including visual confirmation (see "M1 findings" above).** Built `unitree_mujoco` + `unitree_rl_lab`'s `g1_ctrl` on the Orin against a locally-installed `unitree_sdk2` (two installs were needed: one at `/opt/unitree_robotics` per `unitree_mujoco`'s README, one at the default `/usr/local` prefix per `unitree_rl_lab`'s README, since their CMakeLists hardcode different paths). No physical gamepad was available and this Jetson kernel has no `uinput`, so FSM transitions were driven via the `rt/lowstate` relay-with-override pattern instead. Deliverable: screenshots (not a video recording) of the G1 actively walking under the RL policy in MuJoCo, captured headlessly via Xvfb after the Orin's RDP session broke post-reboot.
- **M2a — Full Track A navigation stack: map it, tag it, navigate it. Implemented and validated on the Orin (see the "M2a continued" sections below: 9 of 10, then all tour goals SUCCEEDED).** Scope grew from "goal-following against a pre-built map" to parity with what `home2` does for FRIDA: mapping, area/location tagging, and autonomous goal-reaching, all built on one shared Gazebo foundation. Key design, confirmed against `home2`'s real source (not guessed):
  - `g1_description/urdf/g1.urdf.xacro` wraps a vendored `unitree_ros` G1 URDF (`scripts/vendor_g1.sh` — gitignored output, run on any machine that builds this package): `VelocityControl` + `OdometryPublisher` Gazebo plugins on `pelvis` (G1's real root link — **not** `base_link`), a `gpu_lidar` on the pre-existing `mid360_link` mount point, and the 29 leg/waist/arm joints locked `revolute` -> `fixed` for Track A's static idle stance.
  - `g1_base_bridge`'s Track A now has a watchdog (ported from home2's `cmd_vel_relay.py`) zeroing the Twist if Nav2 stops sending `cmd_vel` — Gazebo's `VelocityControl` holds the last command forever otherwise.
  - `g1_nav_overlay/config/nav2_g1.yaml` + `nav2_g1_sim_overlay.yaml` port `nav2_omni.yaml`'s MPPI/critics structure from home2, deep-merged at launch (same pattern as `nav2_omni.launch.py`'s `_deep_merge()`) and fed to **stock `nav2_bringup`'s `navigation_launch.py`** (not home2's composable-node container — `autostart:true` there does for free what home2 relies on `nav_central.py`, out of scope here, to trigger).
  - `g1_sim_bringup/launch/sim_gz.launch.py` (Fase 4, navigate) and `sim_mapping.launch.py` (Fase 2, map) share the same world/spawn/bridge setup, defaulting to `home2`'s existing `robocup2026_1` map + `arena_robocup2026_1.sdf` world as a first smoke test (no new world-building needed). `localization.launch.py` and `slam.launch.py` from `vendor/home2/navigation/packages/nav_main` are included **unmodified** — both confirmed robot-agnostic by reading their source.
  - Fase 3 (tag) reuses `vendor/home2/navigation/packages/map_context/scripts/map_area_tagger.py` **unmodified** — a standalone PyQt5 app with no ROS/FRIDA coupling, loads a `.pgm`/`.yaml`, exports `areas.json`.
  - See `docs/plan.md`'s own history is superseded here by the fuller ephemeral plan this was implemented from; the concrete file list and design rationale live in git history / the implementation itself.
- **M2b — Real locomotion (Track B, MuJoCo + RL policy). Done in simulation, see "M2b — Track B implemented" below; it is `g1_rl_bridge`, not `g1_base_bridge`, and odometry is still MuJoCo ground truth.** Extend `g1_base_bridge` to target `unitree_rl_lab`'s command interface instead of `VelocityControl`; wire odometry from `SportModeState` into TF/`nav_msgs/Odometry`. Deliverable: the same goal-following demo, but the G1 actually walks via the RL policy in MuJoCo, driven end-to-end by Nav2.
- **M3 — SLAM-built map + autonomous exploration. Done in Gazebo (2026-11-10), see "M3/M4" below: the G1 mapped the arena with its own 3D lidar and navigated it, 14/14 goals.** Simulate the Livox MID-360 (via `pointcloud_to_laserscan`) and/or the D435i on whichever backend proved more stable in M2; run `slam_toolbox` in mapping mode, save a map, rerun Nav2 against it. Deliverable: the G1 autonomously maps and navigates a simulated scenario from scratch.
- **M4 — Integration with home2's `task_manager`/vision/HRI. Navigation part done (2026-11-10): home2's `nav_central`/`semantic_nav` ported into `g1_navigation`, and `task_manager`'s unmodified `NavigationTasks` drives the G1. Vision/HRI not yet run on top.** Wire `task_manager`'s orchestration (first confirm its internal mechanism: BT, state machine, or LLM planner) to emit nav goals to the G1 stack instead of FRIDA's; bring in vision (YOLO/VLM) and HRI (STT/NLP) unmodified, confirming they're robot-agnostic. Deliverable: a GPSR-style demo ("go to X, find Y") running on the G1 sim stack.
- **M5 — Sim-to-real toward the physical G1. Bringup written (`g1_bringup/robot.launch.py`: MID-360 driver, FAST-LIO, `g1_loco_bridge`, navigation), not run on hardware.** Since the hardware is already owned, this phase is in scope for this cycle: replace MuJoCo with real `unitree_sdk2`/DDS on the robot, validate the same `g1_base_bridge` and Nav2 configs against real sensor data and real locomotion latency/safety margins.

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
    g1_nav_overlay/               # nav2_g1.yaml + sim overlay + bridge_gz.yaml + mapper_params (localization/mapping)
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
4. ~~Walking not yet visually confirmed~~ — **Resolved**: confirmed visually via Xvfb + screenshots, see "M1 findings" above.
5. **G1 odometry source for Nav2's TF/`nav_msgs/Odometry`**: the plan originally assumed `SportModeState` would carry position/velocity (true for go2). **Confirmed false for G1**: `unitree_hg::msg::dds_::SportModeState_` only has `fsm_id`/`fsm_mode`/`task_id`/`task_time` — no pose. The real odometry source (state estimator fused from IMU + leg kinematics, possibly published elsewhere, or something `g1_base_bridge` has to compute itself) needs to be identified before M2b's odometry wiring can be written.

## Verification

- M0: `colcon build` with no errors over the full workspace (including the home2 submodule) inside the Docker container (cpu variant).
- M1: screen/video recording of the G1 standing and walking in MuJoCo via teleop and via the RL policy.
- M2a/M2b: `ros2 action send_goal /navigate_to_pose ...` moves the G1 (idle or walking) to the destination in Gazebo/MuJoCo respectively; verify in RViz that costmaps, TF, and the MPPI-planned trajectory look the same as FRIDA's existing demo.
- M3: run SLAM online, save a map, restart Nav2 against the saved map, confirm correct localization (AMCL/slam_toolbox localization mode).
- M4: run a sample GPSR-style task end-to-end and confirm vision/HRI/task_manager required no code changes, only robot configuration.
- M5: physical safety checklist (clear area, accessible emergency stop, conservative velocity limits) before any test on the real G1.

## M2a (Fase 4 smoke test) findings — Orin validation session

Validating the full Fase 4 stack (Gazebo + Nav2 + slam_toolbox localization +
`g1_base_bridge`) on the Orin surfaced several real bugs, in order of
discovery. All code fixes below are committed to the repo (not yet merged
into the Orin's git checkout — only live-pushed there for iteration, per
the "I pull on the Orin myself" workflow).

1. **Gazebo's `OdometryPublisher` plugin stops publishing `/sim/odom`
   shortly after spawn, even while the model keeps moving.** Confirmed via
   `gz topic -e -t /world/.../pose/info`: the model's world pose kept
   changing under `VelocityControl`, but `/sim/odom` published exactly once
   (at spawn) and then never again — not a "publish on pose change" plugin
   (that hypothesis was tested and refuted), just broken on this Gazebo
   Harmonic build. **Fix**: removed the plugin and `/odom` bridge entry
   entirely. `g1_base_bridge` now dead-reckons `/odom` itself by
   integrating the same `vx/vy/wz` it already relays to
   `VelocityControl` — exact for a velocity-controlled kinematic base (no
   slip to model). New params: `spawn_x/y/yaw` (seed the integrator at the
   real spawn pose, matching what the old plugin used to report),
   `odom_out_topic`, `odom_publish_frequency`. Needs `use_sim_time: true`
   passed explicitly now, since it calls `self.get_clock().now()` directly.

2. **Zombie `gz sim` processes silently accumulate across launch/kill
   cycles** if a cleanup `pgrep` pattern doesn't explicitly include
   `gz sim`/`bin/gz` — they don't show up in `ros2 node list`-based
   patterns at all. Symptom: `TF_OLD_DATA`, multiple publishers on
   `/sim/odom` per `gz topic -i`. Always check `pgrep -fa 'gz sim'`
   explicitly in any cleanup, by exact PID, never a broad `-f` pattern in
   the same command that's also launching the next run (self-matching
   bug — split kill and launch into separate shell invocations).

3. **`/dev/shm` accumulates ~500KB of `fastrtps_*` files per DDS
   participant created, forever, since `kill -9` skips graceful DDS
   shutdown.** After ~3 hours of iteration this session it reached ~4GB
   (thousands of files) plus a few-GB `iox-roudi`/iceoryx shared-memory
   daemon left over from M1's direct-DDS testing. Cleaned up
   (`find /dev/shm -maxdepth 1 -name 'fastrtps_*' -delete`, killed
   `iox-roudi`) — turned out NOT to be the real-time-factor root cause
   (see below), but worth doing periodically regardless on long-running
   dev sessions on this box.

4. **Root cause of catastrophic `real_time_factor` (~0.003, i.e. ~300x
   slower than real-time) once the lidar sensor loads: this Orin's libEGL
   can't get hardware acceleration through the X11 platform.**
   `gz sim --headless-rendering` (and GUI mode under Xvfb — same thing,
   Xvfb is a pure software X server with no real GPU/DRM binding) logs
   `libEGL warning: egl: failed to create dri2 screen` and Ogre2 silently
   falls back to software (llvmpipe) rendering. `eglinfo -B` on the Orin
   confirms the Tegra GPU itself is fine and correctly detected — just not
   through the X11 platform:
   ```
   GBM platform:        EGL vendor string: NVIDIA ... Tegra Orin (nvgpu)   [works]
   X11 platform:        eglInitialize failed                               [broken]
   Surfaceless platform: EGL API version: 1.5                              [works]
   ```
   **Fix that works when gz-sim is invoked directly from a shell**: run it
   with `DISPLAY` unset and `EGL_PLATFORM=surfaceless`. Confirmed via a
   from-scratch manual rebuild of the whole stack (gz sim, robot spawn,
   `ros_gz_bridge`, `g1_base_bridge`, `nav_main`'s `localization.launch.py`,
   `nav2_bringup`'s `navigation_launch.py` — each started as its own
   separate background process/shell, not inside one combined
   `ros2 launch` tree): `real_time_factor` held at ~1.0 throughout, lidar
   data flowed, `/odom` published at ~42-45Hz, `odom -> pelvis` TF
   correct.
   (Also tried switching the lidar sensor type from `gpu_lidar` to the CPU
   raycast `lidar` type as an alternative fix, reasoning that CPU raycast
   wouldn't need a render engine at all — **wrong**: in this Gazebo
   version both sensor types still go through Ogre2's scene manager, so
   that sensor never even got its topic advertised in server-only mode
   with no renderer loaded. Reverted to `gpu_lidar`.)

5. **Open problem, not yet resolved: the identical `EGL_PLATFORM=surfaceless`
   + unset-`DISPLAY` fix, applied to the `gz sim` `ExecuteProcess` action
   inside `sim_gz.launch.py` (verified via `/proc/<pid>/environ` on the
   actual spawned process — the env *is* correctly `EGL_PLATFORM=surfaceless`
   with no `DISPLAY` key), still logs the same `failed to create dri2
   screen` warnings and still collapses `real_time_factor` to ~0.003, when
   launched as part of the single combined `ros2 launch
   g1_sim_bringup sim_gz.launch.py` tree.** Same binary, same world, same
   robot, same environment (confirmed by inspection) as the manual test in
   finding 4 — the only difference is being spawned via
   `launch.actions.ExecuteProcess` inside one big `LaunchDescription`
   (alongside ~15 other `IncludeLaunchDescription`/`Node` actions) instead
   of as a standalone background shell process. Not yet root-caused —
   candidates not yet checked: whether `launch`'s own process-group/session
   handling for `ExecuteProcess` differs from a plain `setsid` background
   process in a way that affects Mesa/EGL's platform auto-detection
   (unlikely given the env var is confirmed correct, but not ruled out);
   whether some other action in the same launch tree sets `DISPLAY` back
   via an environment variable action that runs after `gazebo` is
   constructed but before it execs; or something specific to how `launch`
   forks/execs subprocesses (e.g. via a PTY) that EGL's platform probing
   reacts to differently than a raw background `&`.
   **Recommended next step**: don't manage `gz sim` as a `launch.actions.
   ExecuteProcess` inside `sim_gz.launch.py`/`sim_mapping.launch.py` at
   all — start it as an independent process *before* calling
   `ros2 launch` (plain shell script, or a systemd user service), and have
   the launch files only bring up the ROS-side nodes (spawn, bridge,
   base_bridge, localization/slam, nav2), connecting to the
   already-running Gazebo instance. This exactly matches the
   configuration that was proven to work end-to-end in finding 4, and
   avoids re-deriving whatever `launch`-specific quirk is at play.
   `sim_gz.launch.py`/`sim_mapping.launch.py` have NOT been restructured
   this way yet — the `gazebo` `ExecuteProcess` action with the
   `EGL_PLATFORM`/`DISPLAY` env fix is still in both files as committed,
   since the fix is still correct/needed, just not sufficient on its own
   inside the combined launch tree.

## M2a continued — second Orin session: launch-file restructure, costmap/docking bugs, unresolved planning-distance limit

Picked finding 5 back up. Rather than keep chasing the exact `launch`-vs-shell
EGL discrepancy (never root-caused), implemented the recommended workaround
directly: `sim_gz.launch.py` now has a `start_gazebo` launch arg (**default
`false`**). With it false, the file skips creating the `gazebo`
`ExecuteProcess` entirely and assumes Gazebo is already running (started by
hand, or by a future wrapper script, with `DISPLAY` unset and
`EGL_PLATFORM=surfaceless` — see the file's own comments). This is the
*practical* fix; the real root cause of why `ExecuteProcess` specifically
breaks EGL (vs. an identical command backgrounded directly) is still
unknown. One candidate surfaced but not conclusively tested: this Orin has
a real `Xorg`/`gnome-shell` desktop session running (not just our SSH
session), holding `/dev/dri/renderD128` — a second render node
(`renderD129`) is free and used by our Xvfb. Possibly related, not proven.
The staggered-`TimerAction` startup added earlier (suspecting CPU-contention
during gz-sim's EGL init) was **disproven** — `real_time_factor` was
already broken at t=3s, before anything else had started — but the
staggering was left in for unrelated, legitimate "don't race world-loading"
reasons.

With Gazebo started externally and `sim_gz.launch.py ... start_gazebo:=false`
run on top of it, two more real bugs surfaced and were fixed:

6. **A stray `docking_server` process from an much-earlier launch/kill
   cycle this session was never killed (PID far lower than the rest of the
   current batch), and was still alive, claiming the same ROS node name
   (`/docking_server`) as the current run's own docking_server.** Two nodes
   with an identical fully-qualified name on the same ROS graph is
   undefined behavior — this is suspected to be why `ros2 node list` only
   ever printed `/docking_server` and nothing else in this session (even
   though every other node was confirmed alive via `pgrep` and via
   `ros2 action list` correctly listing all of nav2's actions,
   `/navigate_to_pose` included — discovery wasn't actually broken overall,
   just this one node-identity collision). **Fix**: kill every nav2-related
   PID explicitly by PID (not by pattern) before relaunching nav2 — this is
   the same "always check for stragglers by PID, not just by pattern"
   lesson from earlier in this doc, now extended to *every* long-running
   ROS node, not just `gz sim`.

7. **The global costmap doesn't resize to the real map on `autostart`.**
   Confirmed via `/global_costmap/costmap`'s own published `width`/`height`:
   it started at a default 100x100 (5m x 5m) patch at the origin, logged
   `"Robot is out of bounds of the costmap"` and
   `"Sensor origin at (4.20, -11.74) is out of map bounds (0.00, 0.00) to
   (4.98, 4.98)"` — even though `StaticLayer: Resizing costmap to 502 X 410`
   *did* eventually log a few seconds later, matching the real map. After
   the goal-planning failures below triggered many
   `clear_entirely_global_costmap` recovery calls, the live costmap topic
   stopped publishing new data entirely (confirmed via
   `ros2 topic hz` timing out, and a direct rclpy subscription with
   TRANSIENT_LOCAL QoS always returning the exact same stale message,
   frozen at sim-time 420s, no matter how much later it was queried).
   **Fix applied**: kill nav2's processes (including the stray
   docking_server from finding 6) and relaunch nav2 fresh — after a clean
   restart, `/global_costmap/costmap` correctly reported the full 502x410
   grid and kept publishing. Not root-caused *why* the first autostart
   attempt (racing against the stray docking_server and whatever triggered
   the early out-of-bounds warnings) leaves the costmap in this state —
   just confirmed a clean nav2-only restart fixes it. Worth another look if
   it recurs on a from-scratch launch (no stray processes).

8. **Open problem, not resolved: `NavigateToPose`/`ComputePathToPose` fails
   for any goal beyond roughly 1m from the robot's current (localized)
   position, even with a healthy, correctly-sized, freshly-restarted
   costmap.** Reproduced extensively:
   - Goals within ~0.5-0.6m of the robot's current pose: `compute_path_to_pose`
     returns `error_code: 0`, `SUCCEEDED`, `planning_time` ~2ms.
   - Goals ~1m or farther (tested a sequence of points from 1m out to the
     original ~5.5m target at `(3.28, -6.13)`, including several
     intermediate points along the straight-line path): **every single one**
     fails identically — `planner_server` logs
     `"GridBased plugin failed to plan from (sx, sy) to (gx, gy): 'Failed to
     create plan with tolerance of: 0.500000'"`, and the action returns
     `error_code: 208`, `ABORTED`.
   - This is **not** a lethal-obstacle/geometry problem: wrote a small rclpy
     script (subscribing to `/global_costmap/costmap` with matching
     TRANSIENT_LOCAL QoS) to dump actual cost values along the straight-line
     path and in a 2D neighborhood grid around the transition point between
     a succeeding and a failing goal. All sampled cells between the robot
     and the goal are cost 0 (free) or in the 60-100 range (inflation
     gradient, well below the 253/254 lethal/inscribed thresholds) — no
     thin wall or lethal barrier visible in the sampled cells. The robot's
     *own* cell sits in a wide ~99-cost plateau (consistent with being
     roughly 0.35m from a real wall, matching `behavior_server`'s
     independently-reported `"Collision Ahead - Exiting Spin"` when a
     recovery spin was attempted) — but the one successful short-path test
     target was *also* inside that same cost-99 region, so "near a wall"
     alone doesn't explain the success/fail split either.
   - `NavigateToPose`'s feedback during the full end-to-end attempt showed
     `distance_remaining: 0.0` from the very first feedback message (never
     reflecting the true ~5.5m remaining), 16-21 recovery attempts, and the
     robot's position never changing at all (consistent with it never
     having a valid path to follow in the first place).
   - Ruled out so far: costmap size/staleness (retested fresh, same
     result), lethal cells blocking the sampled path, the EGL/launch-tree
     bug (unrelated — this was tested entirely after that was worked
     around). Not yet tried: increasing planner verbosity to DEBUG (the
     Orin's `ros2 param set` CLI hit the same node-discoverability
     oddity as `ros2 lifecycle get`/`ros2 node list`, which blocked
     reconfiguring `planner_server` live without a full relaunch);
     comparing against `vendor/home2`'s real FRIDA footprint/inflation
     values (**done** — FRIDA uses the *same* `inflation_radius: 0.45` with
     an even *larger* rectangular footprint on this exact same map, which
     argues against "our footprint/inflation is too conservative for this
     map's corridors" as the explanation, though doesn't fully rule it out
     for G1's specific position/orientation); visual inspection via RViz
     (not attempted this session — would need Xvfb + a screenshot
     pipeline, same pattern as M1).
   - Also noteworthy, possibly related or possibly a separate effect of
     the same root cause: the robot's slam_toolbox-localized pose
     (`map -> pelvis`, stable at `(3.54, -11.12)`, yaw ~26°) differs from
     the nominal assumed spawn pose
     (`(4.17764, -11.69743)`, yaw -1.989 rad ≈ -114°) by roughly 0.8m and
     ~140° — far more than typical scan-matching jitter. This is the
     *first* time in this whole project that slam_toolbox has had a
     genuinely working lidar to localize against (every earlier session,
     including all of M2a's first pass, had a non-functioning lidar sensor
     — see findings above), so this may be the *first real localization
     result* ever obtained here, and the discrepancy from the assumed
     spawn pose might be either a genuine mislocalization or evidence that
     the assumed `spawn_x/y/yaw` constants don't actually correspond to
     `mapper_params_sim.yaml`'s `map_start_pose` as precisely as assumed.
     Worth checking directly (e.g. temporarily zero cmd_vel entirely,
     fresh spawn, compare `/odom` i.e. dead-reckoned pose against
     `map -> pelvis` immediately after the very first scan, before any
     scan-matching drift could accumulate) before assuming either the
     localization or the planner is "wrong."

**Status at end of this session**: Findings 1-4 (odometry self-computation,
zombie cleanup, EGL/GPU fix) remain fixed and validated. Finding 5 (EGL
breaks specifically inside `ros2 launch`) is *worked around* (not
root-caused) via the new `start_gazebo:=false` default — `sim_gz.launch.py`
now expects Gazebo to already be running. Findings 6-7 (stray docking_server,
costmap not resizing/freezing) are fixed by always killing nav2 PIDs
explicitly before a fresh restart. Finding 8 (planning fails beyond ~1m) is
**unresolved** and is the current blocker on sending a real `NavigateToPose`
smoke-test goal successfully — extensively characterized with reproducible
test data (see above) but not root-caused despite many genuine attempts.
Next session: pick finding 8 back up with the "not yet tried" list above,
starting with checking whether the localization discrepancy noted at the
end of finding 8 is itself the actual cause (e.g., if the robot's believed
pose is subtly wrong, the planner's A* might be exploring from a
degenerate/edge-of-map region that behaves correctly for very short hops but
pathologically for longer ones). At the end of this session, the full stack
(Gazebo with `start_gazebo:=false` externally started, `sim_gz.launch.py`
on top of it, nav2 freshly restarted per finding 7's fix) was left running
on the Orin for inspection — kill by PID (`gz sim`, `ros2 launch`, all
`nav2_*`/`opennav_docking`/`slam_toolbox` processes) before starting a new
attempt from scratch.

## M2a continued — third Orin session (2026-11-05): root causes for findings 4, 5 and 8

**This section supersedes the root-cause claims in findings 4, 5 and 8 above.**

- **The ~0.003 `real_time_factor` was never an EGL or `ros2 launch` problem.** `spawn_z` defaulted to 0, burying the locked legs' collision meshes in the floor, so each physics step cost ~290ms. Raising the live model to z=0.79 with `set_pose` took RTF from 0.0034 to 1.00 immediately, and forcing the NVIDIA EGL vendor beforehand had not changed it. RTF is also 1.0 with `start_gazebo:=true`, inside the combined launch tree, even with the `dri2` warnings present. The launch-file comments about an EGL bug and the `start_gazebo:=false` workaround are therefore obsolete (not yet cleaned up). `spawn_z` now defaults to 0.79.
- **Finding 8 (no plans beyond ~1m) disappeared with a healthy sim.** At spawn_z 0.79, `map -> pelvis` matched the spawn pose and goals planned at up to 6.76m. `NavigateToPose` to `exit` (3.28, -6.13) returned SUCCEEDED with 0 recoveries, ending 0.12m from the goal by Gazebo ground truth. That is one goal, on the URDF that still had collisions.
- **New bug: collisions tip the kinematic base over, which silently breaks dead-reckoned `/odom`.** During a multi-goal tour the model ended at z=1.52 with roll ~70deg, and localization was 1.4m off. A/B with no sensor, same 25s drive command: 37 collisions ended at pitch 82deg, z=0.23; zero collisions stayed level at z=0.79, RTF 1.0. `vendor_g1.sh` now strips all `<collision>` blocks. **Not yet validated: Nav2 goals on the zero-collision model**, because of the GPU problem below.
- **GPU wedge (kernel, needs a reboot).** Starting with one `gz sim` during that tour, the render thread of every later GPU client is stuck in uninterruptible `D` state at `nvgpu_fifo_preempt_tsg`, including a plain `eglinfo`. Symptom: with a sensor model, Gazebo sleeps at 0 CPU with no `/clock` and logs `SceneBroadcaster: Timed out waiting for state`; a world with no sensors runs fine. Cause of the wedge is unknown (`dmesg` needs root). It looks like a URDF or launch bug but is not: the original 37-collision URDF hangs the same way. Check `wchan` of the `ruby` threads before debugging this symptom. Software rendering is not a fallback: Mesa llvmpipe segfaults in `driCreateNewScreen3`, both surfaceless and under Xvfb.
- **Next, after a reboot:** run the five-location tour (kitchen, living_room, exit, inspection_point, start_location) on the zero-collision model, comparing `map -> pelvis` with Gazebo ground truth at each stop.

## M2a continued — after the reboot (2026-11-05): tour results on the zero-collision model

**Corrects the previous section: finding 8 had not fully disappeared.** A healthy sim removed most of it, but NavFn with `use_astar: true` still failed with "Failed to create a plan from potential when a legal potential was found". Same pose and goal: A* failed 3/3, Dijkstra planned. `nav2_g1.yaml` now sets `use_astar: false`.

Two-lap tour with `start_gazebo:=true` and Dijkstra, checked against Gazebo ground truth: **9 of 10 goals SUCCEEDED**, z stayed at 0.79 with no tipping, RTF ~1.0. This also confirms the launch tree is not an RTF problem.

**Still open (intermittent, not root-caused):**
- `exit` aborted once with `NO_VALID_PATH` (208) and 15 recoveries: the planner found no path from around (1.7, -9.6) with "Failed to create plan with tolerance". The same goal passed cleanly on lap 2.
- `inspection_point` succeeded on both laps but needed 9 recoveries, with the same planner warning, and stopped ~0.4m from the goal by ground truth (over `xy_goal_tolerance` 0.35, though within it by `map -> pelvis`).
- Next check: inspect the live `/global_costmap/costmap` at a failure. Candidates, untested: narrow passages closed by robot_radius + inflation, or scan marks from a lidar now ~1.25m high versus a map built at FRIDA's lidar height.

Stop the stack with SIGINT and wait; a straggler node can survive (seen: `waypoint_follower`), so check by PID. Avoid SIGKILL on `gz sim`: unproven as the cause of the GPU wedge, but it has not recurred in this session's graceful restart.

## M2b — Track B implemented: Nav2 walks the G1 in MuJoCo (2026-11-06)

**Result:** three laps over 4 locations of `robocup2026_1`, driven by `NavigateToPose` with the RL policy walking on its legs: **11 of 12 goals SUCCEEDED**, about 14 minutes, no falls. The other goal (`kitchen`) arrived 0.29m away but ran out of time on the final turn. Run this with RViz closed: see "Falls" below.

**This does not validate obstacle avoidance or localization.** The MuJoCo scene has no arena and no lidar: the robot walks on an empty plane while Nav2 plans on the map, using the static map as its only view of the walls, with a fixed `map -> odom`. `/odom` comes from MuJoCo's own ground-truth pose. What it proves is the loop `Nav2 -> cmd_vel -> walking -> odom`.

**To run it:**
```bash
DISPLAY=:10 src/g1_sim_bringup/scripts/start_mujoco.sh
ros2 launch g1_sim_bringup sim_mujoco.launch.py
# Once g1_rl_bridge logs "accepting cmd_vel":
DISPLAY=:10 python3 src/g1_sim_bringup/scripts/release_band.py
```

**What changed, and why:**
- **New package `g1_rl_bridge` (C++, replaces the Python `rl_policy` backend).** The relay was validated in C++, and Python at 1kHz was a risk. It also marks relayed samples in `wireless_remote[0]`, since the relay receives its own writes.
- **The axis mapping in the old plan and bridge was wrong.** From `unitree_rl_lab`'s `velocity_commands` observation: `vx = ly`, `vy = -lx`, `wz = -rx`. The values are direct m/s and rad/s, with no joystick scale.
- **Odometry exists in sim (resolves open question 5 for MuJoCo only).** `unitree_mujoco` publishes the IMU site's world pose on `rt/sportmodestate` using the go2 message type, even for the G1. The real robot still needs its own source.
- **The policy doesn't track linearly.** Measured: 0.2 m/s forward gives nothing, 0.2 rad/s gives 0.02 rad/s. `g1_rl_bridge` compensates each axis with a measured table; 0.2 and 0.3 m/s then track at 0.200 and 0.300.
- **`deploy.yaml` ships the curriculum's starting ranges.** `start_mujoco.sh` widens them (`ang_vel_z` to 1.0, `lin_vel_y` to 0.5), editing the local `third_party/unitree_rl_lab` clone. Without this the robot can't turn. It reaches about 0.41 rad/s.
- **Regulated Pure Pursuit instead of MPPI on this track.** Combining forward and turning couples into sideways drift (0.4 m/s with 0.4 rad/s gave 0.21 m/s to the side), which took MPPI off the path into cells it couldn't recover from. RPP turns on the spot, then walks. Details in `nav2_g1_mujoco_overlay.yaml`.

**Shared fixes, validated on Track A too:**
- **Stale commands:** during one Track A run `/cmd_vel` kept repeating (-0.3, 0.215, 0) at 20Hz with nothing feeding `collision_monitor`, and the robot drove off the map because an arrival-time watchdog never fired. Both bridges now drop commands whose stamp is older than `cmd_vel_timeout`. Fault injection on Track A: stale commands moved it 0.000m, fresh ones 1.178m, and cutting them left it still. What started that repetition was not identified.
- **`default_server_timeout: 200`:** Nav2's 20ms default lost action acknowledgements under load and aborted whole goals with no recovery.
- Track A after all of this: three laps over 5 locations, all goals SUCCEEDED, robot stationary when idle.

**Falls (open):** the robot fell twice, both on the `exit -> start_location` leg with RViz open, about 5 to 6 minutes in. It stumbled during a constant, gentle command (0.25 m/s, no turn). It didn't fall in 7 minutes without Nav2, nor in the 14-minute tour with RViz closed, which frees about 170% CPU. That points at CPU load, likely `g1_ctrl`'s timing, but it is a correlation over few runs. The real-time-factor log meant to confirm it came out empty.

**Still open for the real robot:**
- Use `g1::LocoClient::SetVelocity` rather than the joystick relay; its `duration` argument is a built-in stop. Not written yet.
- Identify the real odometry source, and generate `/scan` from the real MID-360. There is no real-robot bringup.
- The compensation tables are specific to this policy in MuJoCo: they must not be reused on the robot without re-measuring.
- `robot_radius: 0.2` in this overlay is tuned for a sim without physical walls, and is not a safe value near real obstacles.
- ~~Comments about an EGL bug in `sim_gz.launch.py` and `sim_mapping.launch.py`, and the `start_gazebo:=false` default, are obsolete but not cleaned up.~~ Cleaned up (2026-11-07): comments now point at the spawn_z root cause, and `start_gazebo` defaults to true. The `DISPLAY`/`EGL_PLATFORM=surfaceless` env is kept.

## Odometry without ground truth: leg-inertial estimator (2026-11-07, in progress)

Open question 5 for the real robot: `/odom` on Track B came from MuJoCo's ground truth (`rt/sportmodestate`), which the G1 doesn't publish. New package **`g1_leg_odom`** estimates the pelvis pose from `rt/lowstate` only (12 leg joint angles + pelvis IMU), so the same code runs in the sim and on the robot.

- **Method:** IMU orientation, re-based to the spawn yaw. Forward kinematics gives the 8 sole contact points (4 per foot). The stance foot is the one carrying more weight (see "Stance foot by load" below), and its lowest corner is the pivot fixed to the ground; when the pivot changes the anchor moves with it, keeping the position continuous. Velocity is a low-passed finite difference. Planar output (yaw only) on `/odom` + `odom -> base_link`.
- **FK checked against MuJoCo (once, by hand, 2026-11-07):** for 400 random in-range leg poses, `foot_fk` matched MuJoCo's own kinematics of `g1_29dof.xml` to under 1e-5 m (mujoco pip package in a throwaway venv). Not kept as a test, to keep the package free of fixtures; redo it if the leg constants in `leg_odometry.hpp` change.
- **Sim wiring:** the estimator is header-only and runs inside `g1_rl_bridge` (`odom_source:=leg`, now the default of `sim_mujoco.launch.py`). Ground truth moves to `/odom_ground_truth` without TF, and `scripts/odom_error.py` reports the drift. `odom_source:=ground_truth` restores the old setup.
- **Why inside the bridge, not as its own node:** on `lo` without multicast, Cyclone DDS gives each participant one of about 10 auto indices per domain, shared with every DDS process on the machine. The home2 speech container's 6 ROS nodes on domain 0 already take most of them, and a 4th Unitree participant failed with "Failed to find a free participant index". The same limit can make `g1_ctrl` fail at startup. Check `ss -ulpn | grep :74` before blaming the sim.
- **The elastic band breaks it:** while the robot hangs, the lowest point swings in the air and is still treated as fixed; the estimate was 1.2 m off before the robot had moved. `release_band.py` now calls `/g1_rl_bridge/reset_odom` once the feet carry the weight. The robot will have the same problem whenever it is lifted, so call the reset after setting it down.
- **Stance foot by load (2026-11-10): drift went from 3.5% to 0.3% of the distance walked.** The first version took the lowest of the 8 points as the pivot, moving only when another was 5 mm lower. On flat ground that picks badly both ways: the landing foot is as low as the planted one while it still moves, and the margin keeps the old foot as the pivot after it has lifted. The result depended on the margin with no stable optimum (0 mm 2.0%, 2 mm 0.7%, 3 mm 1.5%, 5 mm 3.5%, on one recording), and at 0 mm 2 mrad of encoder noise took it to 12%. Now each foot's vertical force is estimated from its leg's joint torques (`tau_est`, through the leg Jacobian), low-passed 30 ms, and the stance foot changes when the other carries over 60% of the total. Measured on an open-loop `cmd_vel` schedule in MuJoCo (straight, turns in place, arcs, sideways, backwards), no Nav2:
  - Recording 1 (27 m, replayed offline): old 3.5%, new 0.4%. Same result fed at 500 Hz and 250 Hz.
  - Validation run (32 m, new estimator live in `g1_rl_bridge`): 0.10 m at the end, 0.3%. The same data through the height rule: 1.26 m, 3.9%.
  - Standing still it creeps about 1 mm/s.
  - Averaging the contact points' velocities with height weights, instead of one pivot, was worse in every variant tried (4 to 21%).
  - The force estimate ignores the legs' own weight and reads 241 N standing, for a robot of about 343 N: good for comparing the two feet, not as a force measurement. Under 50 N total (robot lifted) it falls back to the height rule. `use_load:=false` forces that rule.
  - **Not verified on the robot:** `tau_est` there comes from motor current, with friction and noise the sim doesn't have. Check it on the gantry: the load should swap sides with each step.
  - Nav2 tour with this version: 5/5 goals, 0.04 m of drift after 33 m (see "Sim loose ends closed" below).
- **Estimator after the relay, not before, in `g1_rl_bridge`.** `g1_ctrl` reads the newest sample on `rt/lowstate`, and until the bridge's copy lands that is MuJoCo's own, with no joystick. With the heavier estimator running before the relay write, the FSM missed the button presses (the robot fell when the band released, three starts in a row) or the policy read zero velocity most of the time (0.1 m in 18 s at 0.3 m/s). With the write first, 0.3 m/s commanded gives about 0.28 m/s, more than the 0.22 measured before, so the compensation tables were re-measured (see "Sim loose ends closed" below).
- Offline tools used for this (recorder of `rt/lowstate` + ground truth to CSV, replay through `leg_odometry.hpp`) are not in the repo; they are on the Orin in `~/hh_test/legodom`.
- In MuJoCo the IMU quaternion is exact, so the sim does not show yaw drift. On the robot it will drift; correcting it needs the MID-360 (`slam_toolbox` or FAST-LIO) or a magnetometer.

## Real-robot bridge: `g1_loco_bridge` (2026-11-07, untested on hardware)

`cmd_vel -> g1::LocoClient::SetVelocity` at `command_rate` (10 Hz), with `command_duration` 0.3 s as the dead-man: if the process dies the robot stops on its own. Stale or missing `cmd_vel` sends zero once, then nothing, so the remote still works while Nav2 is idle. Conservative clamps (`max_vx` 0.3, `max_vy` 0.2, `max_wz` 0.3), `StopMove()` on shutdown, and the same leg odometry with `~/reset_odom`. It does not change the FSM unless `auto_start:=true`; put the robot in walking mode with the remote. It builds but has never run against a robot: first test on the gantry, with the remote's emergency stop in hand. The sim compensation tables are deliberately not used.


## M3/M4 — home2's navigation stack on the G1, mapped with the 3D lidar (2026-11-10)

**Result (Gazebo, `robocup2026_1` arena):** the G1 mapped the arena with its own simulated MID-360, and then navigated it through home2's own interface: **14/14 `go_to_map_area` goals SUCCEEDED** over two laps of 7 locations, localization within 5 cm of Gazebo ground truth at every stop, final position 0.04–0.36 m from the tagged pose. home2's `task_manager` `NavigationTasks`, unmodified, ran `retrieve_areas`, `get_current_pose`, `move_to_location`, `get_path_info` and `plan_patrol` against it. `is_door_open`, `approach_point`, `get_area_for_point`, `query_path` and follow mode on/off also work.

**MuJoCo (Track B) through the same stack** (`sim_mujoco.launch.py` now includes `g1_navigation mode:=mujoco`: static map, `nav_central`, RPP): with `odom_source:=ground_truth`, **4/4 goals SUCCEEDED** walking (kitchen 126 s, living_room 82 s, exit 32 s, start_location 362 s). With the leg odometry (`odom_source:=leg`, the default), the estimate drifted **2.6–4.9% of the distance walked** (0.8 m after ~31 m, yaw error ~0 after the fix below), and `laundry` timed out twice: the robot stopped ~0.35–0.4 m short and RPP alternated between turning in place and driving with the opposite turn. This is the measurement docs/plan.md asked for: without a lidar localizer the leg odometry is not enough for tight goals, which is what FAST-LIO / slam_toolbox are for on the robot.
- `reset_odom` now re-bases `/odom_ground_truth` too: the robot turned 32° while the band lowered it, and `odom_error.py` was reporting that as a constant yaw error.
- `min_approach_linear_velocity` 0.12 → 0.18 in the MuJoCo overlay (the policy barely moves at 0.12 on the final approach). Not enough alone for `laundry` with leg odometry.

**What was built:**

| Piece | Package | Notes |
|---|---|---|
| 3D MID-360 + IMU in Gazebo | `g1_description` | `gpu_lidar` 720×32, −52..+7° (head mount, upside down), 10 Hz; IMU 200 Hz. RTF stayed ~1.0. Bridged as `/livox/lidar`, `/livox/imu`, the real driver's names |
| `base_link` | `g1_description` | Planar footprint, parent of `pelvis` (static, z 0.79). Every odometry source publishes `odom -> base_link`, which every home2 node assumes |
| Lidar pipeline | `g1_perception` | CropBox removes the body (forearms reach 0.38 m in front) → `/livox/points_filtered` → `pointcloud_to_laserscan` 0.10–1.50 m → `/scan` |
| FAST-LIO | `g1_perception` | `lio.launch.py` + `lio_odom_adapter.py` (body pose → planar `odom -> base_link`), `livox_custom_to_pc2`. **Not built or run yet**, see below |
| Navigation | `g1_navigation` | Ports of `nav_central`, `semantic_nav_node` + `semantic/`, `person_goal_smoother`, `table_docker`, `nav_ui`, `map_area_tagger`; `navigation.launch.py` (modes `gz`, `mujoco`, `robot`), `mapping.launch.py`, `slam.launch.py`, `run_nav.sh`, `nav_client_check.py` |
| Real robot | `g1_bringup` | `robot.launch.py`: driver, pipeline, LIO, `g1_loco_bridge` (new `publish_tf`/`odom_topic`), navigation |

**Changes to home2's code in the port** (each file's header lists them): parameters for `base_frame`, `scan_topic`, `required_topics`/`required_frames` (FRIDA waits for its xArm's `link_eef`); RTAB-Map branch removed; `check_door` uses an angular window (±15°) instead of beam indices for FRIDA's 501-beam scan; maps in `G1_MAPS_DIR` (default `src/g1_navigation/maps`) instead of `/workspace/...`; follow mode hides the voxel layer instead of the ZED layer and also sets RPP's speed. **Bug fixed, also present in home2:** `send_nav_goal` assigned `goal_msg.behaivor_tree`, which raises as soon as a behavior tree is passed (`go_to_pose` with a BT, follow).

**Findings, in the order they appeared:**

1. **FRIDA's map doesn't work for the G1.** Localizing against `robocup2026_1` (built with FRIDA's 2D lidars ~0.3 m high) put the robot 0.55 m and 20° off at spawn, and `query_path` failed. The projected 3D scan sees things at other heights. Mapping with the G1's own sensor fixed it: on `g1_robocup2026_1` localization starts exact and stays within 5 cm. 100% of the G1 map's walls are within 0.2 m of FRIDA's, so the map frame is the world frame and FRIDA's area polygons were reused as `areas_g1_robocup2026_1.json`. **Map each arena with the G1.**
2. **Gazebo's `/clock` at ~870 Hz cost every Python node 40–90% of a core** (even `initial_pose`), and slam_toolbox's `map -> odom` lagged ~0.5 s, aborting goals with "Transform data too old". `clock_throttle` (g1_sim_bringup) republishes it at 100 Hz; slam's TF went to 20 Hz. Sim-only tolerance 0.6 s in `nav2_g1_sim_overlay.yaml`.
3. **The arena exit door (~0.8 m) stalled the robot** ("Failed to make progress"): a 0.3 m circle left a 1–2 cell lane. Fixed with a polygon footprint (front 0.40, back −0.20, ±0.24 m, from the sim lidar returns of the arms) and `consider_footprint: true`. The inflation radius then has to cover the circumscribed radius (0.49 m): 0.55, cost scaling 5.0. Exit then took 24–27 s.
4. Lethal blocks in free space left over after localization corrections, never cleared: obstacle layers now raytrace (clear) to 6 m and mark to 3 m.
5. `slam_toolbox` with `respawn` kept a SIGINT'd launch alive; respawn is now robot-only.

**Not done / not verified:**
- **FAST-LIO and the Livox driver were never built**: `third_party/fetch_lio.sh` cloned them but its build was blocked by the session's permission rules. Run it on the Orin, then test `sim_gz.launch.py odom_source:=lio` against ground truth before trusting it on the robot.
- `g1_bringup/robot.launch.py` has only been launched without hardware (it stops with "livox_ros_driver2 not found"). On the robot, check first: the MID-360 orientation (the URDF says upright, the sim models it inverted; `/livox/imu` shows which), the crop box against the real arm pose, the footprint, and `host_ip`/`network_interface`.
- `table_docker` is ported but off by default and untested with legs.
- Vision/HRI on top of the G1 stack, and a full task (GPSR) end to end.
- `nav_central` and `semantic_nav` still use ~50% of a core each in sim (Python TF listener and costmap subscriptions, same code as home2).

**Map → tag → navigate:** see README.md. Maps live in `src/g1_navigation/maps`; `g1_robocup2026_1.posegraph` is 26 MB because the mapping session was long, so remap shorter if repo size matters.

## `./run.sh` and the Nav2 tour with the new leg odometry (2026-11-10)

`./run.sh` at the repo root is now the entry point, in home2's shape: `./run.sh sim|mujoco|robot [--mapping|--nav] [flags]`, plus `--save-map`, `--tagger`, `--ui`, `--check`, `--odom-error`, `--status`, `--stop`, `--build`. See README.md. Run on the Orin: `--help`, `--status`, `--stop` and `./run.sh mujoco -d`, which brought up Xvfb, `unitree_mujoco`, `g1_ctrl` and the launch file, and released the band with the robot standing. **`./run.sh sim` and `./run.sh robot` have not been run**: they only assemble the arguments of launch files that were run by hand before.

**The Nav2 tour in MuJoCo with the load-based leg odometry did not complete, and not because of the odometry.** From the spawn pose on `g1_robocup2026_1`, the planner could not find a path to `laundry` ("Failed to create plan with tolerance", 83 times, 16 goal retries) and the robot only moved through recoveries. Drift over those 8 m of recoveries was 0.17 m (2%). The morning's 4/4 run with ground-truth odometry started with `kitchen` and never planned to `laundry` from the spawn, so the earlier "laundry timed out with leg odometry" may have been this planner failure rather than drift. Root cause found afterwards, see the next section: the robot orbits when told to turn in place.

## Sim loose ends closed (2026-11-10, afternoon)

**This section supersedes "FAST-LIO and the Livox driver were never built" and the open planner failure above.** Nothing here has run on the robot.

- **Why the MuJoCo tour could not plan: the policy does not turn in place.** Told to turn left it also walks backwards (0.2 m/s at full command), an orbit of about 0.5 m radius. The spawn has 0.55 m of clearance and MuJoCo has no physical walls, so turning towards `laundry` left the robot 0.10 m from a wall of the map, inside the inflated cells, with no plan to anywhere. The map itself is fine: every goal is reachable from the spawn at radii up to 0.3 m.
- **In-place turn compensation in `g1_rl_bridge`** (`turn_left_*`, `turn_right_*`, used when the commanded linear speed is under `turn_in_place_below`). Measured with raw commands, 12 s each: a forward command cancels the backwards walk on left turns (1.0 rad/s with 0.3 m/s forward gives 0.40 rad/s and under 0.03 m/s), and the policy turns right at about half the rate of left, up to 0.22 rad/s. With the tables: 0.16 rad/s asked gives +0.18 and -0.16, orbit radius 0.05 to 0.12 m. Arcs (turning while walking) still use the old symmetric table and were not re-measured.
- **`vx` table re-measured:** the policy does not step under a command of 0.25 m/s (the old table had 0.109 m/s there). Now 0.25 asked gives 0.28, 0.15 gives 0.19, -0.2 gives -0.22. `vy` matched the old table and was left alone.
- **MuJoCo, Nav2 tour on leg odometry: 5/5 goals** (laundry, kitchen, living_room, exit, start_location) in 250 s of walking, no planner failures, leg odometry 0.04 m off after 33 m. One lap, one run.
- **Livox driver and FAST-LIO now build on the Orin** with `third_party/fetch_lio.sh`: the driver needed `-DDISTRO_ROS`, and FAST-LIO pins C++14 where Jazzy's rclcpp needs C++17. `livox_custom_to_pc2` builds once the overlay is sourced (`./run.sh --build` sources it; clear `build/g1_perception` if it was configured before the overlay existed).
- **FAST-LIO in Gazebo was diverging, fixed.** Gazebo's `gpu_lidar` fills rays with no return with `inf` and marks the cloud dense, so the body crop kept 4113 of them per scan; FAST-LIO's voxel filter then kept about 140 points and the estimate left by metres within the first goal. `lidar_pipeline` now drops non-finite points first (`drop_no_return`). After that, on isolated moves it matched the exact motion to 1-2 cm, and a Nav2 tour with `--odom lio` did 5/5 goals with 0.03 m of odometry error after 27 m. This is the sim sensor (instantaneous scans, ideal IMU): it shows the wiring is right, not how FAST-LIO behaves on a walking robot.
- **`./run.sh sim` verified:** `--mapping` (map grew from 73x183 to 254x223 cells over three goals), `--save-map`, `--map <that map>` with two goals, `--odom lio`, `--no-nav`, `--stop`. `--save-map` now calls `map_saver_cli` itself: `/slam_toolbox/save_map` returned 255 in Gazebo and left a pose graph with no `.pgm`. While mapping, a goal has to be inside the map built so far ("outside bounds" otherwise), and `--check --tour` needs tagged areas, which a new map does not have yet.
- **Loose reading, not followed up:** on the freshly saved test map the second goal reported SUCCEEDED with `map -> base_link` 0.7 m from it, from a single `tf2_echo` line that may have been stale.
- **`./run.sh robot` still not run.** `robot.launch.py --print` resolves with the lidar overlay sourced; it was not launched, since the Orin sits on FRIDA's network.
