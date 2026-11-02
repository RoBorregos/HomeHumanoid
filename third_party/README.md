# third_party/

Unitree repos cloned directly (not as git submodules, since they move fast
and are consumed as external tools rather than build dependencies of this
workspace). Ignored by `.gitignore`.

## unitree_mujoco (M1, M2b)

```bash
git clone https://github.com/unitreerobotics/unitree_mujoco third_party/unitree_mujoco
```

Usage (outside ROS 2):

```bash
cd third_party/unitree_mujoco
./unitree_mujoco -r g1 -s scene_terrain.xml
```

## unitree_rl_lab (M1, M2b)

```bash
git clone https://github.com/unitreerobotics/unitree_rl_lab third_party/unitree_rl_lab
```

Requires Isaac Lab (>=2.3, 5.0+ recommended) only if **training**; for
sim-to-sim validation against `unitree_mujoco`, the `g1_ctrl` binary is
enough:

```bash
cd third_party/unitree_rl_lab
./g1_ctrl --network <iface>
```

**Before implementing the `rl_policy` variant of `g1_base_bridge` (Track B,
M2b):** read `deploy/robots/g1/real_g1_bridge.py` and `controller_bridge.py`
directly in this cloned repo to confirm the exact topic/type of the velocity
command interface — see open question #1 in `docs/plan.md`. The
`"vel vx vy vyaw"` format currently used in `src/g1_base_bridge` is an
unverified placeholder.

## unitree_ros (URDF source, M2a)

```bash
git clone https://github.com/unitreerobotics/unitree_ros third_party/unitree_ros
```

Copy `robots/g1_description/g1_29dof.urdf` (+ meshes) into
`src/g1_description/urdf/` following its README.md instructions.
