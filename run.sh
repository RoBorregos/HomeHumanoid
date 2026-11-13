#!/usr/bin/env bash
# HomeHumanoid entry point, the counterpart of home2's ./run.sh.
set -eo pipefail

REPO="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
LOG="${G1_LOG_DIR:-/tmp/g1nav}"
SCRIPTS="$REPO/src/g1_sim_bringup/scripts"
RUN_NAV="$REPO/src/g1_navigation/scripts/run_nav.sh"

usage() {
  cat << EOF
Usage: ./run.sh <area> [task] [flags]         in the area's container (docker/<area>)
       ./run.sh [target] [task] [flags]       natively, on this computer

Areas (one container each, like home2's ./run.sh):
  navigation         Lidar pipeline, FAST-LIO, slam_toolbox, Nav2, nav_central; on the
                     real G1 also the MID-360 driver and g1_loco_bridge.
  simulation         Gazebo and MuJoCo (development machines only).
  vision, hri, ...   Forwarded to home2's ./run.sh (vendor/home2).
  See ./run.sh navigation --help and ./run.sh simulation --help.

Targets (native, where the G1 runs):
  sim                Gazebo: kinematic base, simulated MID-360. Mapping and navigation.
  mujoco             MuJoCo: the RL walking policy on its legs. Static map, no lidar.
  robot              The real G1 (UNTESTED on hardware: gantry and remote e-stop first).

Tasks:
  --nav              Localization on a saved map + Nav2 + nav_central (default).
  --mapping          slam_toolbox mapping, with Nav2 on the growing map (sim, robot).

Tools (with the stack already running, unless noted):
  --save-map NAME    Save the map being built into src/g1_navigation/maps.
  --tagger           Tag areas on a saved map (no stack needed).
  --ui               nav_ui: initial pose, goals, save map.
  --check [ARGS]     Call the services home2's task_manager uses (--tour all, ...).
  --odom-error       mujoco: leg odometry drift against MuJoCo's ground truth.
  --release-band     mujoco: lower and release the elastic band (the mujoco target does it).
  --status           What is running.
  --stop             Stop everything this script started.
  --build [PKGS]     colcon build, all packages or the ones given.
  --clean            Delete build/, install/ and log/.

Flags:
  --map NAME         Map to localize on (default: \$MAP_NAME, else g1_robocup2026_1).
  --headless         sim: no Gazebo window (default when there is no DISPLAY).
  --no-nav           Robot or simulator only, no navigation stack.
  --nav-ui           Also open nav_ui.
  --odom SRC         Odometry source. sim: bridge | lio. mujoco: leg | ground_truth.
                     robot: lio | leg.
  --display :N       mujoco: X display for its window. Without one, starts Xvfb on :99.
  --keep-band        mujoco: leave the robot hanging from the elastic band.
  --iface IF         robot: network interface to the G1 (default eth0).
  --host-ip IP       robot: this computer's address on the G1 network.
  --auto-start       robot: let the bridge put the G1 in walking mode itself.
  -d                 Detached: return once it is up. Logs in $LOG, stop with --stop.
  key:=value         Passed through to the launch file.

Examples:
  ./run.sh sim --mapping --headless      then  ./run.sh --save-map lab  and  ./run.sh --tagger
  ./run.sh sim --map lab
  ./run.sh mujoco -d                     then  ./run.sh --check --tour all
  ./run.sh robot --mapping --iface eth0 --host-ip 192.168.123.164
  ./run.sh --stop
EOF
}

setup_env() {
  set +u
  source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
  # Overridable for the containers (/workspace/install, /home/ros/lio_ws).
  local lio="${G1_LIO_SETUP:-$REPO/third_party/lio_ws/install/setup.bash}" ws="${G1_INSTALL_DIR:-$REPO/install}"
  [ -f "$lio" ] && source "$lio"
  [ -f "$ws/setup.bash" ] || { echo "Not built yet: ./run.sh --build" >&2; exit 1; }
  source "$ws/setup.bash"
  export G1_MAPS_DIR="${G1_MAPS_DIR:-$REPO/src/g1_navigation/maps}"
  mkdir -p "$LOG"
}

# Everything started here goes in its own session, recorded so --stop can find it.
track() { echo "$2" >> "$LOG/run.pids"; echo "$1 (pid $2)"; }

# Anchored on the start of the command line, so a shell or ssh command that only
# mentions one of these names is not taken for the process itself.
ours() {
  pgrep -u "$(id -u)" -af 'ros2 launch g1_|unitree_mujoco|g1_ctrl|gz sim|g1_rl_bridge|g1_loco_bridge' \
    | grep -E '^[0-9]+ ([^ ]*python3 [^ ]*ros2 launch g1_|[^ ]*unitree_mujoco|[^ ]*g1_ctrl|gz sim|ruby [^ ]*gz sim|[^ ]*/g1_(rl|loco)_bridge)' || true
}

stop_all() {
  local pids=""
  [ -f "$LOG/run.pids" ] && pids="$(cat "$LOG/run.pids")"
  for f in "$LOG"/mujoco.pid "$LOG"/g1_ctrl.pid; do [ -f "$f" ] && pids="$pids $(cat "$f")"; done
  # SIGINT to the whole session of each, as Ctrl-C would. Not SIGKILL: nodes that
  # don't shut DDS down leave shared memory behind, and it is a suspect for the
  # Orin's GPU hang (docs/plan.md).
  for p in $pids; do kill -INT -- "-$p" 2>/dev/null || kill -INT "$p" 2>/dev/null || true; done
  for _ in $(seq 20); do
    local alive=""
    for p in $pids; do kill -0 "$p" 2>/dev/null && alive="$alive $p"; done
    [ -z "$alive" ] && break
    sleep 1
  done
  for p in $pids; do kill -TERM -- "-$p" 2>/dev/null || kill -TERM "$p" 2>/dev/null || true; done
  rm -f "$LOG/run.pids" "$LOG/mujoco.pid" "$LOG/g1_ctrl.pid" "$LOG/xvfb.pid"
  sleep 1
  local left; left="$(ours)"
  if [ -n "$left" ]; then
    echo "Still running (not started by this script, or not stopping):"; echo "$left" | cut -c1-120
    return 1
  fi
  echo "Stopped."
}

# Runs a launch file in its own session: in the foreground until Ctrl-C, or
# detached with -d.
launch() {
  local name="$1"; shift
  echo "ros2 launch $*"
  setsid ros2 launch "$@" > "$LOG/$name.log" 2>&1 < /dev/null &
  LAUNCH_PID=$!
  track "$name, log $LOG/$name.log" "$LAUNCH_PID"
}

follow() {
  [ "$DETACHED" = 1 ] && { echo "Detached. ./run.sh --status, ./run.sh --stop"; return; }
  trap 'echo; stop_all; exit 0' INT TERM
  tail -n +1 -f "$LOG/$1.log" --pid "$LAUNCH_PID" &
  wait $! 2>/dev/null || true
  stop_all || true
}

TARGET="" TASK="--nav" MAP="${MAP_NAME:-g1_robocup2026_1}" DETACHED=0 NAV=true NAV_UI=false
ODOM="" HEADLESS="" MJ_DISPLAY="${DISPLAY:-}" KEEP_BAND=0 IFACE="" HOST_IP="" AUTO_START=false
PASS=()

[ $# -eq 0 ] && { usage; exit 0; }
source "$REPO/lib.sh"
case "$1" in
  navigation|simulation)
    area="$1"; shift
    cd "$REPO/docker/$area" && exec bash ./run.sh "$@" "$(detect_env_type)" ;;
  vision|hri|manipulation|integration|display|zed)
    cd "$REPO/vendor/home2" && exec ./run.sh "$@" ;;
esac
while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    sim|mujoco|robot) TARGET="$1" ;;
    --nav|--mapping) TASK="$1" ;;
    --map) MAP="${2:?--map NAME}"; shift ;;
    --headless) HEADLESS=1 ;;
    --no-nav) NAV=false ;;
    --nav-ui) NAV_UI=true ;;
    --odom) ODOM="${2:?--odom SRC}"; shift ;;
    --display) MJ_DISPLAY="${2:?--display :N}"; shift ;;
    --keep-band) KEEP_BAND=1 ;;
    --iface) IFACE="${2:?--iface IF}"; shift ;;
    --host-ip) HOST_IP="${2:?--host-ip IP}"; shift ;;
    --auto-start) AUTO_START=true ;;
    -d) DETACHED=1 ;;
    *:=*) PASS+=("$1") ;;
    --stop) mkdir -p "$LOG"; stop_all; exit $? ;;
    --status)
      r="$(ours)"; [ -n "$r" ] && echo "$r" | cut -c1-140 || echo "Nothing running."
      ls -t "$LOG"/*.log 2>/dev/null | head -5 | sed 's/^/log: /'; exit 0 ;;
    --clean) rm -rf "$REPO/build" "$REPO/install" "$REPO/log"; echo "Cleaned."; exit 0 ;;
    --build)
      shift; set +u; source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
      [ -f "$REPO/third_party/lio_ws/install/setup.bash" ] && source "$REPO/third_party/lio_ws/install/setup.bash"
      cd "$REPO"; if [ $# -gt 0 ]; then exec colcon build --packages-select "$@"; else exec colcon build; fi ;;
    --save-map|--tagger|--ui|--check)
      [ "$1" = "--ui" ] && export MAP_NAME="$MAP"
      exec "$RUN_NAV" "$@" ;;
    --odom-error) setup_env; shift; exec python3 "$SCRIPTS/odom_error.py" "$@" ;;
    --release-band) setup_env; DISPLAY="${MJ_DISPLAY:-:99}" exec python3 "$SCRIPTS/release_band.py" ;;
    *) echo "Unknown argument: $1" >&2; echo "./run.sh --help" >&2; exit 1 ;;
  esac
  shift
done

[ -n "$TARGET" ] || { echo "Missing target: sim, mujoco or robot. ./run.sh --help" >&2; exit 1; }
setup_env
if [ -n "$(ours)" ]; then
  echo "Something is already running, stop it first (./run.sh --stop):" >&2; ours | cut -c1-120 >&2; exit 1
fi
rm -f "$LOG/run.pids"
export MAP_NAME="$MAP"
SLAM=localization; [ "$TASK" = "--mapping" ] && SLAM=mapping
ARGS=(map_name:="$MAP" nav:="$NAV")
[ -n "$ODOM" ] && ARGS+=(odom_source:="$ODOM")

case "$TARGET" in
  sim)
    GUI=true; { [ -n "$HEADLESS" ] || [ -z "${DISPLAY:-}" ]; } && GUI=false
    launch sim_launch g1_sim_bringup sim_gz.launch.py slam:="$SLAM" gui:="$GUI" nav_ui:="$NAV_UI" "${ARGS[@]}" "${PASS[@]}"
    follow sim_launch ;;

  mujoco)
    [ "$TASK" = "--mapping" ] && { echo "mujoco has no lidar: map with ./run.sh sim --mapping" >&2; exit 1; }
    if [ -z "$MJ_DISPLAY" ] || ! xdpyinfo -display "$MJ_DISPLAY" > /dev/null 2>&1; then
      MJ_DISPLAY=:99
      if ! xdpyinfo -display :99 > /dev/null 2>&1; then
        setsid Xvfb :99 -screen 0 1280x800x24 > "$LOG/xvfb.log" 2>&1 < /dev/null &
        track "Xvfb :99 (no usable display)" $!; sleep 2
      fi
    fi
    DISPLAY="$MJ_DISPLAY" "$SCRIPTS/start_mujoco.sh" "$LOG"
    sleep 3
    launch mujoco_launch g1_sim_bringup sim_mujoco.launch.py "${ARGS[@]}" "${PASS[@]}"
    echo "Waiting for the walking policy..."
    for _ in $(seq 60); do grep -q "accepting cmd_vel" "$LOG/mujoco_launch.log" && break; sleep 1; done
    grep -q "accepting cmd_vel" "$LOG/mujoco_launch.log" || { echo "g1_rl_bridge never got to the walking policy, see $LOG/mujoco_launch.log" >&2; stop_all; exit 1; }
    # g1_ctrl can miss the bridge's button presses; releasing the band then drops the robot.
    grep -q "FixStand to Velocity" "$LOG/g1_ctrl.log" || { echo "g1_ctrl is not in Velocity, see $LOG/g1_ctrl.log. Stopping: the robot would fall." >&2; stop_all; exit 1; }
    if [ "$KEEP_BAND" = 0 ]; then
      DISPLAY="$MJ_DISPLAY" python3 "$SCRIPTS/release_band.py"
    else
      echo "Robot left on the band: ./run.sh --release-band --display $MJ_DISPLAY"
    fi
    follow mujoco_launch ;;

  robot)
    [ -n "$IFACE" ] && ARGS+=(network_interface:="$IFACE")
    [ -n "$HOST_IP" ] && ARGS+=(host_ip:="$HOST_IP")
    launch robot_launch g1_bringup robot.launch.py slam:="$SLAM" auto_start:="$AUTO_START" nav_ui:="$NAV_UI" "${ARGS[@]}" "${PASS[@]}"
    follow robot_launch ;;
esac
