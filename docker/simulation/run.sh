#!/bin/bash
# Simulation area: ./run.sh simulation [task] [flags]   (see usage below)
source ../../lib.sh
set -e

ARGS=("$@")
TASK="${ARGS[0]:-}"
ENV_TYPE="${*: -1}"
[[ "$ENV_TYPE" =~ ^(cpu|cuda|l4t)$ ]] || ENV_TYPE="$(detect_env_type)"

COMPOSE_FILE="docker-compose.yaml"
NAME=homehumanoid-simulation

usage() {
  cat <<USAGE
Usage: ./run.sh simulation [task] [flags]

Tasks:
  (none)             Open a shell in the simulation container
  --gz               Gazebo: arena world, G1 (kinematic base), simulated MID-360 + IMU,
                     ros_gz bridge and odometry. Navigation: ./run.sh navigation --nav
  --mujoco           MuJoCo: the G1 walking with Unitree's RL policy (g1_ctrl +
                     g1_rl_bridge), released from the band. Navigation:
                     ./run.sh navigation --nav --mode mujoco
  --release-band     mujoco: lower and release the band (after --keep-band)
  --odom-error       mujoco: leg odometry drift against MuJoCo's ground truth
  --status           What is running in the container

Flags:
  --headless         gz: no Gazebo window (default when there is no DISPLAY)
  --odom SRC         gz: bridge | lio (lio: FAST-LIO in the navigation container
                     owns odom; pass the same --odom there). mujoco: leg | ground_truth
  --keep-band        mujoco: leave the robot hanging from the elastic band
  key:=value         Passed to the launch file (spawn_x:=..., world:=...)
  -d                 Detached (logs: docker logs -f $NAME, docker/simulation/logs)
  --build            colcon build the simulation packages first
  --build-image      Rebuild the image
  --recreate | --stop | --down | --clean

Examples:
  ./run.sh simulation --gz --build -d
  ./run.sh navigation --nav
  ./run.sh simulation --mujoco -d && ./run.sh navigation --nav --mode mujoco
USAGE
}

case $TASK in -h|--help|help) usage; exit 0 ;; esac

parse_common_flags "$COMPOSE_FILE" "${ARGS[@]}"

FLAGS=() PASS=() TOOL_ARGS=()
i=1
while [ $i -lt ${#ARGS[@]} ]; do
  a="${ARGS[$i]}"
  case $a in
    --headless|--keep-band) FLAGS+=("$a") ;;
    --odom) i=$((i+1)); FLAGS+=(--odom "${ARGS[$i]}") ;;
    *:=*) PASS+=("$a") ;;
    -d|--build|--build-image|--recreate|--clean|cpu|cuda|l4t) ;;
    *) TOOL_ARGS+=("$a") ;;
  esac
  i=$((i+1))
done

#_________________________SETUP_________________________

setup_common_env simulation "$ENV_TYPE"
mkdir -p cache/gz

# No $VARS in these: COMMAND goes through .env, where compose would expand them.
BASE_SETUP="source /opt/ros/jazzy/setup.bash && source /usr/local/bin/cyclonedds_setup.sh"
# Only the sim side. frida_gz_sim for the arena world; selected, not up-to, as
# its manifest also names home2's manipulation and vision packages.
PACKAGES="frida_interfaces frida_constants frida_gz_sim g1_description g1_leg_odom g1_base_bridge \
g1_rl_bridge g1_nav_overlay g1_perception g1_navigation g1_sim_bringup"
COLCON="MAKEFLAGS=-j4 colcon build --symlink-install --parallel-workers 2 --packages-select $PACKAGES \
--cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"
if [ "$BUILD" = true ]; then
  SETUP="$BASE_SETUP && $COLCON && source install/setup.bash"
else
  SETUP="$BASE_SETUP && if [ -f install/setup.bash ]; then source install/setup.bash; else echo Not built: ./run.sh simulation --build; exit 1; fi"
fi
SHELL_SETUP="$BASE_SETUP; if [ -f install/setup.bash ]; then source install/setup.bash; fi"
[ "$BUILD" = true ] && SHELL_SETUP="$SETUP"

# The repo's ./run.sh does the sequencing (Gazebo, then spawn and bridges;
# MuJoCo, g1_ctrl, the bridge, then the band) inside the container, with
# navigation and the lidar pipeline left to the navigation container.
RUN_SH="cd /workspace/src && exec ./run.sh"
RUN=""
case $TASK in
  --gz) RUN="$RUN_SH sim --no-nav perception:=false ${FLAGS[*]} ${PASS[*]}" ;;
  --mujoco) RUN="$RUN_SH mujoco --no-nav ${FLAGS[*]} ${PASS[*]}" ;;
  --release-band|--odom-error) TOOL="cd /workspace/src && ./run.sh $TASK ${TOOL_ARGS[*]}" ;;
  --status) TOOL="ps -eo pid,etime,rss,args --sort=-rss | grep -E '[g]z sim|[u]nitree_mujoco|[g]1_ctrl|[r]os2 launch|[g]1_' | cut -c1-150" ;;
  ""|-d|--build|--build-image|--recreate|--clean|cpu|cuda|l4t) ;;
  *) usage; exit 1 ;;
esac

#_________________________RUN_________________________

if [ -n "${TOOL:-}" ]; then
  [ -n "$BUILD_IMAGE" ] && docker compose build
  in_container "$NAME" "$COMPOSE_FILE" simulation "$SETUP && $TOOL"
elif [ -n "$RUN" ]; then
  if container_running "$NAME"; then
    echo "$NAME is already running. ./run.sh simulation --stop first." >&2
    exit 1
  fi
  add_or_update_variable .env COMMAND "$SETUP && $RUN"
  docker compose up $DETACHED $BUILD_IMAGE
else
  if ! container_running "$NAME" || [ -n "$BUILD_IMAGE" ]; then
    add_or_update_variable .env COMMAND "sleep infinity"
    docker compose up -d $BUILD_IMAGE
  fi
  tty="-i"; [ -t 0 ] && tty="-it"
  docker exec $tty "$NAME" bash -c "$SHELL_SETUP; exec bash"
fi
