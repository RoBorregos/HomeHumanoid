# vendor/

This directory holds dependencies on other repos managed as **git
submodules**, pinned to a known commit (they do not auto-update).

## home2

Still needs to be added manually (a git operation the team decides when to
run; not done as part of this scaffold):

```bash
git submodule add https://github.com/RoBorregos/home2 vendor/home2
git submodule update --init --recursive
```

Packages from `home2` this repo expects to consume by ROS name once the
submodule is initialized (see docs/plan.md):

- `frida_interfaces` — custom messages/services, reusable unchanged.
- `task_manager` — competition task orchestration (deferred, not this phase).
- `vision` (`object_detector_2d`, `vision_general`, `moondream_run`) — robot-agnostic (deferred).
- `hri` (`speech`, `nlp`, `embeddings`, `display`) — robot-agnostic (deferred).
- `navigation/nav_main` — **actively included, not just a pattern reference**:
  `g1_sim_bringup`'s launch files `IncludeLaunchDescription` its
  `localization.launch.py` and `slam.launch.py` **unmodified** (both are
  robot-agnostic — confirmed by reading their source, no FRIDA-specific
  imports). `nav2_omni.yaml`'s structure is ported (not reused as-is) into
  `src/g1_nav_overlay/config/nav2_g1.yaml`.
- `simulation/frida_gz_sim` — **actively consumed**: `arena_robocup2026_1.sdf`
  is used directly as the default Gazebo world (no new world-building needed
  for the first pass). The Gazebo plugin pattern (`VelocityControl` +
  `OdometryPublisher` on the base link) is ported into `g1.urdf.xacro`.
- `navigation/packages/map_context` — **actively consumed**: the
  `robocup2026_1` map (`.pgm`/`.yaml`/`.posegraph`/`.data`) is used directly
  for the first localization smoke test. `map_area_tagger.py` (area/location
  tagging) is run as-is — it's a standalone PyQt5 app with no ROS/FRIDA
  coupling at all.

Packages from home2 that are NOT reused (specific to FRIDA's holonomic base):
`navigation/omnidriver`, `manipulation/*` (xArm6-specific), `robot_description/frida_description`,
`navigation/nav_main`'s `nav_central.py` (task-orchestration-coupled — see docs/plan.md
for why G1's Nav2 launch uses stock `nav2_bringup` autostart instead).
