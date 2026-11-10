#!/usr/bin/env bash
# G1 navigation entry point, the counterpart of home2's docker/navigation/run.sh.
#
#   run_nav.sh --mapping  [gz|robot]      slam_toolbox mapping + Nav2 on the growing map
#   run_nav.sh --save-map <name>          save the running map into src/g1_navigation/maps
#   run_nav.sh --tagger                   map_area_tagger (areas_<name>.json)
#   run_nav.sh --nav      [gz|mujoco|robot] [map]   localization + Nav2 + nav_central (MAP_NAME)
#   run_nav.sh --ui                       nav_ui (initial pose, goals, save map)
#   run_nav.sh --check    [check args]    nav_client_check.py (the services nav_tasks.py uses)
#
# Only the navigation side: start the robot first (sensors + odometry), e.g.
#   ros2 launch g1_sim_bringup sim_gz.launch.py nav:=false      (Gazebo)
#   ros2 launch g1_bringup robot.launch.py nav:=false           (real G1)
# or let those launch files include navigation themselves (their default).
#
# Maps are read from and saved to $G1_MAPS_DIR, default src/g1_navigation/maps in
# this repo, so a saved map can be committed and is used without rebuilding.
set -eo pipefail

if [ -n "${G1_REPO:-}" ]; then
  REPO="$G1_REPO"
else
  # Works from the source tree and from install/g1_navigation/lib/g1_navigation.
  REPO="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && git rev-parse --show-toplevel 2>/dev/null || true)"
  [ -n "$REPO" ] || REPO="$HOME/HomeHumanoid"
fi
export G1_MAPS_DIR="${G1_MAPS_DIR:-$REPO/src/g1_navigation/maps}"
MAP_NAME="${MAP_NAME:-robocup2026_1}"

set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
# Overridable for the navigation container (/home/ros/lio_ws, /workspace/install).
LIO_SETUP="${G1_LIO_SETUP:-$REPO/third_party/lio_ws/install/setup.bash}"
[ -f "$LIO_SETUP" ] && source "$LIO_SETUP"
source "${G1_INSTALL_DIR:-$REPO/install}/setup.bash"
set -u

usage() { sed -n '2,19p' "$0" | sed 's/^# \{0,1\}//'; exit 1; }

case "${1:-}" in
  --mapping)
    exec ros2 launch g1_navigation mapping.launch.py mode:="${2:-gz}" ;;
  --nav)
    exec ros2 launch g1_navigation navigation.launch.py mode:="${2:-gz}" map_name:="${3:-$MAP_NAME}" ;;
  --save-map)
    name="${2:?map name}"
    mkdir -p "$G1_MAPS_DIR/areas"
    base="$G1_MAPS_DIR/$name"
    echo "Serializing pose graph -> $base.posegraph/.data"
    ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$base'}"
    echo "Saving occupancy grid -> $base.yaml/.pgm"
    # map_saver_cli directly, not /slam_toolbox/save_map: that service runs the
    # same tool with a 2 s wait for /map and returned 255 ("Failed to spin map
    # subscription") in Gazebo, leaving a pose graph with no .pgm/.yaml.
    sim_time="$(ros2 param get /slam_toolbox use_sim_time 2>/dev/null | grep -o 'True\|False' || echo False)"
    ros2 run nav2_map_server map_saver_cli -f "$base" --ros-args \
      -p save_map_timeout:=10.0 -p map_subscribe_transient_local:=true -p use_sim_time:="$sim_time"
    [ -f "$base.yaml" ] && [ -f "$base.pgm" ] || { echo "Map not saved: no $base.yaml/.pgm" >&2; exit 1; }
    ls -la "$base".*
    echo "Next: run_nav.sh --tagger (save areas as $G1_MAPS_DIR/areas/areas_$name.json), then MAP_NAME=$name run_nav.sh --nav" ;;
  --tagger)
    exec ros2 run g1_navigation map_area_tagger.py ;;
  --ui)
    exec ros2 run g1_navigation nav_ui.py --ros-args -p map_name:="$MAP_NAME" ;;
  --check)
    shift; exec ros2 run g1_navigation nav_client_check.py "$@" ;;
  *) usage ;;
esac
