# third_party/

Unitree repos cloned directly (not as git submodules, since they move fast
and are consumed as external tools rather than build dependencies of this
workspace). Ignored by `.gitignore`.

## unitree_sdk2 (M1, M2b — C++ SDK, shared dependency)

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2 third_party/unitree_sdk2
```

Needs to be built and installed **twice**, at two different prefixes,
because `unitree_mujoco` and `unitree_rl_lab`'s CMakeLists hardcode
different locations:

```bash
# for unitree_mujoco (its README recommends this prefix)
cd third_party/unitree_sdk2 && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
sudo make install

# for unitree_rl_lab's deploy/robots/g1_29dof (hardcodes /usr/local)
cd third_party/unitree_sdk2 && mkdir build_local && cd build_local
cmake .. -DBUILD_EXAMPLES=OFF
sudo make install
sudo ldconfig   # required, or g1_ctrl fails with "error while loading shared libraries: libddsc.so"
```

Also install:

```bash
sudo apt install libyaml-cpp-dev libspdlog-dev libboost-all-dev libglfw3-dev libeigen3-dev libfmt-dev
```

## unitree_mujoco (M1, M2b)

```bash
git clone https://github.com/unitreerobotics/unitree_mujoco third_party/unitree_mujoco
```

Also download a MuJoCo release matching the host arch (e.g. aarch64 on the
Orin) from https://github.com/google-deepmind/mujoco/releases, extract to
`~/.mujoco/`, and symlink it:

```bash
ln -s ~/.mujoco/mujoco-<version> third_party/unitree_mujoco/simulate/mujoco
```

Build:

```bash
cd third_party/unitree_mujoco/simulate && mkdir build && cd build
cmake .. && make -j4
```

**Configuration is via `simulate/config.yaml`, not CLI flags** (the
`-r g1 -s scene_terrain.xml` style flags referenced in older docs/plans don't
exist in the current binary). For G1 sim2sim, per `unitree_rl_lab`'s own
deploy instructions:

```yaml
robot: "g1"
robot_scene: "scene_29dof.xml"
domain_id: 0              # NOT unitree_mujoco's own default of 1
interface: "lo"
use_joystick: 0           # 1 only if a real gamepad is plugged into /dev/input/js0
enable_elastic_band: 1    # G1 needs lifting like H1
```

Run: `./unitree_mujoco` (reads `config.yaml` from the working directory).

## unitree_rl_lab (M1, M2b)

```bash
git clone https://github.com/unitreerobotics/unitree_rl_lab third_party/unitree_rl_lab
```

Requires Isaac Lab (>=2.3, 5.0+ recommended) only if **training**; for
sim-to-sim validation against `unitree_mujoco`, build just the deploy
controller:

```bash
cd third_party/unitree_rl_lab/deploy/robots/g1_29dof
mkdir build && cd build
cmake .. && make
```

**The vendored `deploy/thirdparty/onnxruntime-linux-x64-1.22.0/` is x86_64
and will not run on aarch64 (Jetson/Orin).** Replace it:

```bash
cd third_party/unitree_rl_lab/deploy/thirdparty
rm -rf onnxruntime-linux-x64-1.22.0
curl -sLO https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-aarch64-1.22.0.tgz
tar xf onnxruntime-linux-aarch64-1.22.0.tgz && rm onnxruntime-linux-aarch64-1.22.0.tgz
# then patch the path in deploy/robots/g1_29dof/CMakeLists.txt:
sed -i 's/onnxruntime-linux-x64-1.22.0/onnxruntime-linux-aarch64-1.22.0/g' ../robots/g1_29dof/CMakeLists.txt
```

Run (both `unitree_mujoco` and `g1_ctrl` must use the same `--network`/
`interface`, e.g. both `lo`):

```bash
cd third_party/unitree_rl_lab/deploy/robots/g1_29dof/build
./g1_ctrl --network lo
```

**Controlling the FSM (`Passive -> FixStand -> Velocity`) without a
gamepad:** see docs/plan.md, "M1 findings". There is no `/command` topic —
the joystick state is bytes embedded in `rt/lowstate` itself
(`wireless_remote` field). This Jetson's kernel has no `uinput`, so a
virtual-gamepad device isn't an option here; the mechanism that worked is a
small relay that subscribes to the real `rt/lowstate` and republishes it
immediately with only `wireless_remote` overridden (preserves real
motor/IMU telemetry, avoids a second publisher corrupting it). This is also
the production design for `g1_base_bridge`'s Track B — see
`src/g1_base_bridge/g1_base_bridge/base_bridge_node.py`.

## unitree_ros (URDF source, M2a)

```bash
git clone https://github.com/unitreerobotics/unitree_ros third_party/unitree_ros
```

Copy `robots/g1_description/g1_29dof.urdf` (+ meshes) into
`src/g1_description/urdf/` following its README.md instructions. Note:
`unitree_mujoco/unitree_robots/g1/` already bundles G1 MJCF scene files with
meshes (used above for the MuJoCo track) — `unitree_ros` is only needed for
the separate URDF used by Gazebo/Track A.

## unitree_sdk2_python (M2b, `g1_base_bridge` Track B)

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2_python third_party/unitree_sdk2_python
cd third_party/unitree_sdk2_python && pip3 install -e .
```

Needed by `g1_base_bridge`'s `rl_policy` backend (the hybrid rclpy +
raw-DDS bridge, see its module docstring) — not yet installed/tested on the
Orin as of this writing; the exact `unitree_sdk2py` import paths used in
that file should be verified against whatever version gets installed.
