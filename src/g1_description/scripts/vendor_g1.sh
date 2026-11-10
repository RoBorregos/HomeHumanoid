#!/usr/bin/env bash
# Vendors the Unitree G1 URDF + meshes from unitreerobotics/unitree_ros into
# this package. The output (g1_29dof_raw.urdf + meshes/) is gitignored —
# re-run this script on any machine that needs to build g1_description,
# same pattern as third_party/ (see third_party/README.md).
#
# Two transforms are applied to the raw URDF on vendor, documented here so
# they're reproducible instead of hand-edited:
#   1. mesh filename="meshes/X" -> filename="package://g1_description/meshes/X"
#      (the upstream file uses bare relative paths, which robot_state_publisher
#      can't resolve).
#   2. The 29 actuated joints (legs/waist/arms) are changed from
#      type="revolute" to type="fixed", keeping their URDF-default origin as
#      the idle stance. Track A (Gazebo kinematic base) has no controller for
#      them; left revolute, the model would collapse under gravity. Fixed
#      sensor-mount joints (mid360_joint, d435_joint, imu_*) are untouched.
#   3. Every <collision> block is removed. Track A's base is velocity-driven,
#      not physically simulated: g1_base_bridge dead-reckons /odom from cmd_vel,
#      so any contact force silently breaks odometry. With collisions, the
#      locked legs dragged on the floor and tipped the model over mid-run, and
#      buried at spawn_z:=0 they cost ~290ms per physics step. Obstacle
#      avoidance is Nav2's job, from /scan.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
URDF_DIR="${PKG_DIR}/urdf"
MESHES_DIR="${PKG_DIR}/meshes"
WORK_DIR="$(mktemp -d)"
trap 'rm -rf "${WORK_DIR}"' EXIT

echo "Cloning unitreerobotics/unitree_ros (sparse: robots/g1_description only)..."
git clone --depth 1 --filter=blob:none --sparse https://github.com/unitreerobotics/unitree_ros "${WORK_DIR}/unitree_ros"
git -C "${WORK_DIR}/unitree_ros" sparse-checkout set robots/g1_description

SRC="${WORK_DIR}/unitree_ros/robots/g1_description"
if [ ! -f "${SRC}/g1_29dof.urdf" ]; then
  echo "ERROR: g1_29dof.urdf not found at ${SRC} — upstream layout may have changed." >&2
  exit 1
fi

echo "Copying meshes..."
rm -rf "${MESHES_DIR}"
mkdir -p "${MESHES_DIR}"
cp -r "${SRC}/meshes/." "${MESHES_DIR}/"

echo "Rewriting mesh paths to package:// and locking actuated joints to fixed..."
RAW_OUT="${URDF_DIR}/g1_29dof_raw.urdf"
mkdir -p "${URDF_DIR}"
sed 's#filename="meshes/#filename="package://g1_description/meshes/#g' \
  "${SRC}/g1_29dof.urdf" > "${RAW_OUT}"

# Joints locked to fixed for Track A's static idle stance (kinematic virtual
# base — see docs/plan.md). Sensor/IMU mount joints are deliberately excluded.
ACTUATED_JOINTS=(
  left_hip_pitch_joint left_hip_roll_joint left_hip_yaw_joint
  left_knee_joint left_ankle_pitch_joint left_ankle_roll_joint
  right_hip_pitch_joint right_hip_roll_joint right_hip_yaw_joint
  right_knee_joint right_ankle_pitch_joint right_ankle_roll_joint
  waist_yaw_joint waist_roll_joint waist_pitch_joint
  left_shoulder_pitch_joint left_shoulder_roll_joint left_shoulder_yaw_joint
  left_elbow_joint left_wrist_roll_joint left_wrist_pitch_joint left_wrist_yaw_joint
  right_shoulder_pitch_joint right_shoulder_roll_joint right_shoulder_yaw_joint
  right_elbow_joint right_wrist_roll_joint right_wrist_pitch_joint right_wrist_yaw_joint
)

for joint in "${ACTUATED_JOINTS[@]}"; do
  sed -i -E "s#(<joint name=\"${joint}\" type=)\"revolute\"#\1\"fixed\"#" "${RAW_OUT}"
done

perl -0pi -e 's#\s*<collision>.*?</collision>##gs' "${RAW_OUT}"

echo "Vendored: ${RAW_OUT}"
echo "Meshes:   ${MESHES_DIR} ($(find "${MESHES_DIR}" -type f | wc -l) files)"
echo "Done. g1.urdf.xacro includes g1_29dof_raw.urdf directly."
