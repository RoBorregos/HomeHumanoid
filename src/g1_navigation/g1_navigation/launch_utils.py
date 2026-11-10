"""Helpers shared by g1_navigation's launch files (no ROS imports at module level)."""

import json
import math
import os
import tempfile

import yaml

# Nav2 overlay deep-merged on nav2_g1.yaml per mode (all in g1_nav_overlay/config).
NAV2_OVERLAYS = {
    "gz": ["nav2_g1_sim_overlay.yaml"],
    "mujoco": ["nav2_g1_mujoco_overlay.yaml"],
    "robot": ["nav2_g1_robot_overlay.yaml"],
}
MODES = tuple(NAV2_OVERLAYS)


def check_mode(mode):
    if mode not in MODES:
        raise RuntimeError(f"mode must be one of {MODES}, got {mode!r}")
    return mode


def deep_merge(base, overlay):
    """Recursively merge overlay into base (overlay wins). Same as home2's
    nav2_omni.launch.py _deep_merge()."""
    for k, v in overlay.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            deep_merge(base[k], v)
        else:
            base[k] = v
    return base


def merged_nav2_params(config_dir, mode, extra=(), name=None):
    """nav2_g1.yaml + the mode's overlay(s) + `extra` files, written to a temp
    file whose path is returned."""
    with open(os.path.join(config_dir, "nav2_g1.yaml")) as f:
        params = yaml.safe_load(f)
    for overlay in [*NAV2_OVERLAYS[check_mode(mode)], *extra]:
        with open(os.path.join(config_dir, overlay)) as f:
            deep_merge(params, yaml.safe_load(f) or {})
    suffix = "_".join([mode, *[os.path.splitext(e)[0] for e in extra]])
    path = os.path.join(tempfile.gettempdir(), name or f"nav2_g1_{suffix}_merged.yaml")
    with open(path, "w") as f:
        yaml.safe_dump(params, f, default_flow_style=False, sort_keys=False)
    return path


def yaw_from_quat(qz, qw, qx=0.0, qy=0.0):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def start_pose_from_areas(maps_dir, map_name, area="start_location", key="safe_place"):
    """(x, y, yaw) of areas_<map>.json[area][key], or None.

    areas.json poses are [x, y, z, qx, qy, qz, qw] (home2 map_area_tagger)."""
    path = os.path.join(maps_dir, "areas", f"areas_{map_name}.json")
    try:
        with open(path) as f:
            p = json.load(f)[area][key]
    except (OSError, KeyError, ValueError, TypeError):
        return None
    if len(p) < 7:
        return None
    return float(p[0]), float(p[1]), yaw_from_quat(p[5], p[6], p[3], p[4])
