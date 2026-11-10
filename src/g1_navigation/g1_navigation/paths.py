"""Where the G1's maps live.

G1_MAPS_DIR (set by scripts/run_nav.sh to src/g1_navigation/maps in the repo)
wins, so maps saved while mapping land in the source tree and are used
without rebuilding. Otherwise the installed share/g1_navigation/maps.
Layout, same as home2's map_context/maps:
    <name>.yaml + <name>.pgm          occupancy grid (nav2 map_server)
    <name>.posegraph + <name>.data    slam_toolbox graph (localization)
    areas/areas_<name>.json           tagged areas (nav_central, semantic_nav)
"""

import os


def maps_dir():
    env = os.environ.get("G1_MAPS_DIR", "")
    if env:
        return env
    from ament_index_python.packages import get_package_share_directory

    return os.path.join(get_package_share_directory("g1_navigation"), "maps")
