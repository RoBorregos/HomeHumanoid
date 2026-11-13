#!/usr/bin/env bash
# Helpers shared by ./run.sh and docker/<area>/run.sh, after home2's lib.sh.

# Areas with their own container in this repo. vision, hri, manipulation, ...
# still come from home2 (./run.sh forwards them to vendor/home2/run.sh).
G1_AREAS="navigation simulation"
HOME2_AREAS="vision hri manipulation integration display zed"

if [[ -n "${__G1_LIB_SOURCED:-}" ]]; then
  return 0
fi
__G1_LIB_SOURCED=1
G1_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

detect_env_type() {
  if [ -f /etc/nv_tegra_release ]; then
    echo l4t
  elif command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
    echo cuda
  else
    echo cpu
  fi
}

add_or_update_variable() {
  local file=$1 variable=$2 value=$3 escaped
  escaped=$(printf '%s\n' "$value" | sed -e 's/[&/\]/\\&/g')
  if grep -q "^${variable}=" "$file" 2>/dev/null; then
    sed -i "s|^${variable}=.*|${variable}=${escaped}|" "$file"
  else
    printf '%s=%s\n' "$variable" "$value" >> "$file"
  fi
}

# home2's base image (ROS Jazzy, the ros user, CycloneDDS; on l4t also CUDA,
# OpenCV and torch). The navigation image is built on it so that, on the G1's
# Orin, it shares its layers with home2's vision and hri images.
ensure_home2_base() {
  local env_type=$1 image="roborregos/home2:${1}_base"
  docker image inspect "$image" >/dev/null 2>&1 && return 0
  echo "$image not found locally. Trying docker pull..."
  docker pull "$image" && return 0
  local compose="$G1_LIB_DIR/vendor/home2/docker/${env_type}.yaml"
  [ -f "$compose" ] || { echo "No $compose: run git submodule update --init --recursive" >&2; return 1; }
  echo "Building $image from vendor/home2 (long the first time)..."
  USER_UID="$(id -u)" USER_GID="$(id -g)" docker compose -f "$compose" build
}

# Flags every area takes. Sets DETACHED, BUILD, BUILD_IMAGE; handles the
# container-only ones itself.
parse_common_flags() {
  local compose="docker compose -f $1"
  shift
  DETACHED="" BUILD="" BUILD_IMAGE=""
  for arg in "$@"; do
    case $arg in
      -d) DETACHED="-d" ;;
      --build) BUILD=true ;;
      --build-image) BUILD_IMAGE="--build" ;;
      --clean) rm -rf build install log; echo "Cleaned build/ install/ log/" ;;
      --recreate) $compose down ;;
      --down) $compose down; exit 0 ;;
      --stop) $compose stop; exit 0 ;;
    esac
  done
}

# .env read by docker-compose.yaml, the same variables for every area.
setup_common_env() {
  local area=$1 env_type=$2
  echo "" > .env
  [ -f /etc/cyclonedds.env ] && source /etc/cyclonedds.env
  add_or_update_variable .env CYCLONE_INTERFACE "${CYCLONE_INTERFACE:-}"
  add_or_update_variable .env LOCAL_USER_ID "$(id -u)"
  add_or_update_variable .env LOCAL_GROUP_ID "$(id -g)"
  add_or_update_variable .env IMAGE_NAME "roborregos/homehumanoid:${area}-${env_type}"
  # Both containers must share these to see each other (docs: README step 0).
  add_or_update_variable .env ROS_DOMAIN_ID "${ROS_DOMAIN_ID:-0}"
  add_or_update_variable .env GZ_PARTITION "${GZ_PARTITION:-}"
  add_or_update_variable .env MAP_NAME "${MAP_NAME:-g1_robocup2026_1}"
  add_or_update_variable .env DISPLAY "${DISPLAY:-}"
  if [ "$env_type" = cpu ]; then
    add_or_update_variable .env DOCKER_RUNTIME runc
  else
    add_or_update_variable .env DOCKER_RUNTIME nvidia
  fi
  mkdir -p build install log logs
  # GUI windows (nav_ui, tagger, Gazebo, MuJoCo) on the host's X server.
  if [ -n "${DISPLAY:-}" ] && command -v xhost >/dev/null 2>&1; then
    xhost +local: >/dev/null 2>&1 || true
  fi
}

container_running() {
  [ -n "$(docker ps -q -f "name=^$1$")" ]
}

# Runs a command in the area's container: in the running one if it is up,
# otherwise in a throwaway one.
in_container() {
  local name=$1 compose=$2 service=$3 cmd=$4 tty="-i"
  [ -t 0 ] && tty="-it"
  if container_running "$name"; then
    docker exec $tty "$name" bash -c "$cmd"
  else
    docker compose -f "$compose" run --rm "$service" bash -c "$cmd"
  fi
}
