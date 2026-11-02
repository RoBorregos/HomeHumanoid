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
- `task_manager` — competition task orchestration (M4).
- `vision` (`object_detector_2d`, `vision_general`, `moondream_run`) — robot-agnostic (M4).
- `hri` (`speech`, `nlp`, `embeddings`, `display`) — robot-agnostic (M4).
- `navigation/nav_main` — reference for `nav2_standard.yaml`/`omni_config`/`mapper_params_sim.yaml`,
  which are adapted (not reused as-is) in `src/g1_nav_overlay/config/`.
- `simulation/frida_gz_sim` — pattern reference for `src/g1_description/`
  (xacro wrapping + Gazebo plugins) and `worlds/world_from_map.py`.

Packages from home2 that are NOT reused (specific to FRIDA's holonomic base):
`navigation/omnidriver`, `manipulation/*` (xArm6-specific), `robot_description/frida_description`.
