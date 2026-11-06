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

cd "$ROOT/third_party/unitree_rl_lab/deploy/robots/g1_29dof/build"
setsid nohup ./g1_ctrl --network lo > "$LOG/g1_ctrl.log" 2>&1 < /dev/null &
echo $! > "$LOG/g1_ctrl.pid"
echo "mujoco=$(cat "$LOG/mujoco.pid") g1_ctrl=$(cat "$LOG/g1_ctrl.pid")"
