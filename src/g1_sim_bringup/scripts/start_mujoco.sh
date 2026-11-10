#!/usr/bin/env bash
# Starts unitree_mujoco and g1_ctrl for Track B on the given display (default :10,
# the remote desktop session). Run sim_mujoco.launch.py afterwards.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
LOG="${1:-/tmp/g1nav}"; mkdir -p "$LOG"
export DISPLAY="${DISPLAY:-:10}"

cd "$ROOT/third_party/unitree_mujoco/simulate/build"
setsid nohup ./unitree_mujoco > "$LOG/mujoco.log" 2>&1 < /dev/null &
echo $! > "$LOG/mujoco.pid"
sleep 5

# deploy.yaml ships the training curriculum's starting ranges, which barely turn
# the robot: 0.2 rad/s yields about 0.02 rad/s. The policy reaches about 0.4 rad/s
# when commanded 1.0, so widen what g1_ctrl lets through. Idempotent.
sed -i -E 's/ang_vel_z: \[-0\.2, 0\.2\]/ang_vel_z: [-1.0, 1.0]/; s/lin_vel_y: \[-0\.3, 0\.3\]/lin_vel_y: [-0.5, 0.5]/' \
  "$ROOT/third_party/unitree_rl_lab/deploy/robots/g1_29dof/config/policy/velocity/v0/params/deploy.yaml"

cd "$ROOT/third_party/unitree_rl_lab/deploy/robots/g1_29dof/build"
setsid nohup ./g1_ctrl --network lo > "$LOG/g1_ctrl.log" 2>&1 < /dev/null &
echo $! > "$LOG/g1_ctrl.pid"
echo "mujoco=$(cat "$LOG/mujoco.pid") g1_ctrl=$(cat "$LOG/g1_ctrl.pid")"
