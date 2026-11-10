# vendor/

This directory holds dependencies on other repos managed as **git
submodules**, pinned to a known commit (they do not auto-update).

## home2

```bash
git submodule update --init --recursive
```

`vendor/home2` is the reference for what the G1 should be able to do: the
same tasks FRIDA runs. Packages consumed by ROS name:

- `frida_interfaces`, `frida_constants` — messages/services and the
  `/navigation/*` names. Reused unchanged, so `task_manager`'s
  `nav_tasks.py` talks to the G1 exactly as to FRIDA.
- `simulation/frida_gz_sim` — `arena_robocup2026_1.sdf` is the default
  Gazebo world.
- `task_manager`, `vision`, `hri` — robot-agnostic, to run on top of the G1
  navigation stack (docs/plan.md, M4).

**Ported, not consumed** (copied into `src/g1_navigation` and adapted, see
each file's header for the home2 commit and the changes):

- `navigation/nav_main`: `nav_central.py` (base frame / scan / required
  frames as parameters, no `link_eef`, no RTAB-Map, angular door window),
  `semantic_nav_node.py` + `nav_main/semantic/`, `person_goal_smoother.py`,
  `table_docker.py`, the behavior trees, and the launch structure
  (`general_navigation` / `mapping` / `localization` / `slam`).
  `nav2_omni.yaml`'s structure lives on in `src/g1_nav_overlay/config/nav2_g1.yaml`.
- `navigation/map_context`: `nav_ui.py` and `map_area_tagger.py` (saving
  into `src/g1_navigation/maps`), and the `robocup2026_1` map + areas.

Not reused (FRIDA hardware): `navigation/omnidriver`, `sllidar_ros2`,
`ira_laser_tools` (the G1 has one 3D lidar, see `g1_perception`),
`manipulation/*` (xArm6), `robot_description/frida_description`.
