#!/bin/bash
# Navigation area: ./run.sh navigation [task] [flags]   (see usage below)
source ../../lib.sh
set -e

ARGS=("$@")
TASK="${ARGS[0]:-}"
ENV_TYPE="${*: -1}"
[[ "$ENV_TYPE" =~ ^(cpu|cuda|l4t)$ ]] || ENV_TYPE="$(detect_env_type)"

COMPOSE_FILE="docker-compose.yaml"
NAME=homehumanoid-navigation

usage() {
  cat <<USAGE
Usage: ./run.sh navigation [task] [flags]

Tasks:
  (none)             Open a shell in the navigation container
  --nav              Localization on a saved map + Nav2 + nav_central (default map: \$MAP_NAME)
  --mapping          slam_toolbox mapping (Nav2 on the growing map in sim)
  --save-map NAME    Save the map being built into src/g1_navigation/maps
  --tagger           Tag areas on a saved map
  --ui               nav_ui (initial pose, goals)
  --check [ARGS]     The services home2's task_manager uses (--tour all, ...)
  --rviz             RViz

Flags:
  --mode MODE        gz | mujoco | robot (default: \$G1_MODE, else gz). What the stack
                     runs on: the simulation container (gz, mujoco) or the real G1.
  --map NAME         Map to localize on (default: \$MAP_NAME, else g1_robocup2026_1)
  --odom SRC         gz: bridge | lio. robot: lio | leg
  --nav-ui           Also open nav_ui
  --iface IF         robot: interface on the G1 network (default eth0)
  --host-ip IP       robot: this computer's IP there (default 192.168.123.164)
  --auto-start       robot: let g1_loco_bridge put the G1 in walking mode
  key:=value         Passed to the launch file
  -d                 Detached (logs: docker logs -f $NAME)
  --build            colcon build the navigation packages first
  --build-image      Rebuild the image
  --recreate | --stop | --down | --clean

Examples:
  ./run.sh simulation --gz -d && ./run.sh navigation --nav --build
  ./run.sh navigation --check --tour all
  ./run.sh navigation --mapping --mode robot           (on the G1's Orin)
USAGE
}

case $TASK in -h|--help|help) usage; exit 0 ;; esac

parse_common_flags "$COMPOSE_FILE" "${ARGS[@]}"

MODE="${G1_MODE:-gz}" MAP="${MAP_NAME:-g1_robocup2026_1}" ODOM="" NAV_UI=false
IFACE="" HOST_IP="" AUTO_START=false TOOL_ARGS=() PASS=()
i=1
while [ $i -lt ${#ARGS[@]} ]; do
  a="${ARGS[$i]}"
  case $a in
    --mode) i=$((i+1)); MODE="${ARGS[$i]}" ;;
    --map) i=$((i+1)); MAP="${ARGS[$i]}" ;;
    --odom) i=$((i+1)); ODOM="${ARGS[$i]}" ;;
    --nav-ui) NAV_UI=true ;;
    --iface) i=$((i+1)); IFACE="${ARGS[$i]}" ;;
    --host-ip) i=$((i+1)); HOST_IP="${ARGS[$i]}" ;;
    --auto-start) AUTO_START=true ;;
    *:=*) PASS+=("$a") ;;
    -d|--build|--build-image|--recreate|--clean|cpu|cuda|l4t) ;;
    *) TOOL_ARGS+=("$a") ;;
  esac
  i=$((i+1))
done
[[ "$MODE" =~ ^(gz|mujoco|robot)$ ]] || { echo "--mode must be gz, mujoco or robot" >&2; exit 1; }
export MAP_NAME="$MAP"

#_________________________SETUP_________________________

setup_common_env navigation "$ENV_TYPE"
ensure_home2_base "$ENV_TYPE"
add_or_update_variable .env BASE_IMAGE "roborregos/home2:${ENV_TYPE}_base"

# No \$VARS in these: COMMAND goes through .env, where compose would expand them.
BASE_SETUP="source /opt/ros/jazzy/setup.bash && source /usr/local/bin/cyclonedds_setup.sh && source /home/ros/lio_ws/install/setup.bash"
# frida_interfaces/constants come from vendor/home2, as for home2's own areas.
COLCON="MAKEFLAGS=-j4 colcon build --symlink-install --parallel-workers 2 \
--packages-up-to g1_bringup g1_navigation g1_perception --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"
if [ "$BUILD" = true ]; then
  SETUP="$BASE_SETUP && $COLCON && source install/setup.bash"
else
  SETUP="$BASE_SETUP && if [ -f install/setup.bash ]; then source install/setup.bash; else echo Not built: ./run.sh navigation --build; exit 1; fi"
fi
SHELL_SETUP="$BASE_SETUP; if [ -f install/setup.bash ]; then source install/setup.bash; fi"
[ "$BUILD" = true ] && SHELL_SETUP="$BASE_SETUP && $COLCON && source install/setup.bash"

LAUNCH_ARGS=(mode:="$MODE" map_name:="$MAP" nav_ui:="$NAV_UI")
[ -n "$ODOM" ] && LAUNCH_ARGS+=(odom_source:="$ODOM")
[ -n "$IFACE" ] && LAUNCH_ARGS+=(network_interface:="$IFACE")
[ -n "$HOST_IP" ] && LAUNCH_ARGS+=(host_ip:="$HOST_IP")
[ "$MODE" = robot ] && LAUNCH_ARGS+=(auto_start:="$AUTO_START")

RUN_NAV=/workspace/src/src/g1_navigation/scripts/run_nav.sh
RUN=""
case $TASK in
  --nav) RUN="ros2 launch g1_bringup nav_area.launch.py slam:=localization ${LAUNCH_ARGS[*]} ${PASS[*]}" ;;
  --mapping)
    [ "$MODE" = mujoco ] && { echo "mujoco has no lidar: map with --mode gz" >&2; exit 1; }
    RUN="ros2 launch g1_bringup nav_area.launch.py slam:=mapping ${LAUNCH_ARGS[*]} ${PASS[*]}" ;;
  --save-map|--tagger|--ui|--check) TOOL="$RUN_NAV $TASK ${TOOL_ARGS[*]}" ;;
  --rviz) TOOL="rviz2" ;;
  ""|-d|--build|--build-image|--recreate|--clean|cpu|cuda|l4t) ;;
  *) usage; exit 1 ;;
esac

#_________________________RUN_________________________

if [ -n "${TOOL:-}" ]; then
  # Tools talk to a running stack (or none, for the tagger): no new launch.
  [ -n "$BUILD_IMAGE" ] && docker compose build
  in_container "$NAME" "$COMPOSE_FILE" navigation "$SETUP && $TOOL"
elif [ -n "$RUN" ]; then
  if container_running "$NAME"; then
    echo "$NAME is already running. ./run.sh navigation --stop first, or use a tool (--check, --ui...)." >&2
    exit 1
  fi
  add_or_update_variable .env COMMAND "$SETUP && exec $RUN"
  docker compose up $DETACHED $BUILD_IMAGE
else
  # Shell: reuse the container, or start an idle one.
  if ! container_running "$NAME" || [ -n "$BUILD_IMAGE" ]; then
    add_or_update_variable .env COMMAND "sleep infinity"
    docker compose up -d $BUILD_IMAGE
  fi
  tty="-i"; [ -t 0 ] && tty="-it"
  docker exec $tty "$NAME" bash -c "$SHELL_SETUP; exec bash"
fi
