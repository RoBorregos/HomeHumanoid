#!/usr/bin/env bash
# Clones and builds the MID-360 / LIO dependencies into a separate colcon
# overlay, third_party/lio_ws, pinned to known commits:
#   - Livox-SDK2         (C++ SDK, installed into lio_ws/sdk, no sudo)
#   - livox_ros_driver2  (MID-360 driver, publishes /livox/lidar + /livox/imu)
#   - FAST_LIO (ROS2)    (lidar-inertial odometry used by g1_perception)
#
# Same convention as the rest of third_party/: cloned, not submodules, and
# gitignored. Source the overlay before building HomeHumanoid:
#   source /opt/ros/jazzy/setup.bash
#   source third_party/lio_ws/install/setup.bash
#   colcon build ...
#
# Usage: third_party/fetch_lio.sh [--no-build]
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${HERE}/lio_ws"
SDK_PREFIX="${WS}/sdk"
JOBS="${JOBS:-6}"

LIVOX_SDK2_URL=https://github.com/Livox-SDK/Livox-SDK2.git
LIVOX_SDK2_REV=c0796f04c143143899c87a773d9f6b7136453c0b
LIVOX_DRIVER_URL=https://github.com/Livox-SDK/livox_ros_driver2.git
LIVOX_DRIVER_REV=21445540f0d100dc86a7e6df312dd70bbdb4afdf
FAST_LIO_URL=https://github.com/hku-mars/FAST_LIO.git
FAST_LIO_REV=cc3c9e8b1469b33b8072764d6db2348ed6dc597f   # branch ROS2

clone() {  # url rev dir
  if [ ! -d "$3/.git" ]; then
    git clone --quiet "$1" "$3"
  fi
  git -C "$3" fetch --quiet origin "$2" 2>/dev/null || true
  git -C "$3" checkout --quiet "$2"
  git -C "$3" submodule update --init --recursive --quiet
}

mkdir -p "${WS}/src" "${WS}/deps"
clone "$LIVOX_SDK2_URL" "$LIVOX_SDK2_REV" "${WS}/deps/Livox-SDK2"
clone "$LIVOX_DRIVER_URL" "$LIVOX_DRIVER_REV" "${WS}/src/livox_ros_driver2"
clone "$FAST_LIO_URL" "$FAST_LIO_REV" "${WS}/src/FAST_LIO"

# livox_ros_driver2 ships package_ROS1.xml / package_ROS2.xml and expects its
# build.sh to copy the right one into place.
cp -f "${WS}/src/livox_ros_driver2/package_ROS2.xml" "${WS}/src/livox_ros_driver2/package.xml"

# GCC 13 (Ubuntu 24.04) no longer pulls <cstdint> in transitively.
for f in $(grep -rl "uint8_t\|uint32_t" "${WS}/deps/Livox-SDK2/sdk_core" --include=*.h || true); do
  grep -q "<cstdint>" "$f" || sed -i '0,/#include/s//#include <cstdint>\n#include/' "$f"
done

[ "${1:-}" = "--no-build" ] && exit 0

cmake -S "${WS}/deps/Livox-SDK2" -B "${WS}/deps/Livox-SDK2/build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${SDK_PREFIX}" \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON >/dev/null
cmake --build "${WS}/deps/Livox-SDK2/build" -j"${JOBS}"
cmake --install "${WS}/deps/Livox-SDK2/build" >/dev/null

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
cd "${WS}"
CMAKE_PREFIX_PATH="${SDK_PREFIX}:${CMAKE_PREFIX_PATH:-}" \
LIBRARY_PATH="${SDK_PREFIX}/lib:${LIBRARY_PATH:-}" \
CPATH="${SDK_PREFIX}/include:${CPATH:-}" \
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 \
  -DLIVOX_LIDAR_SDK_LIBRARY="${SDK_PREFIX}/lib/liblivox_lidar_sdk_shared.so" \
  -DLIVOX_LIDAR_SDK_INCLUDE_DIR="${SDK_PREFIX}/include" \
  -DCMAKE_INSTALL_RPATH="${SDK_PREFIX}/lib" -DCMAKE_BUILD_RPATH="${SDK_PREFIX}/lib" \
  --parallel-workers 2 --event-handlers console_cohesion+

echo "Done. source ${WS}/install/setup.bash"
