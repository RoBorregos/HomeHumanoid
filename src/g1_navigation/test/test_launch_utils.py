import json
import math
import os

import yaml

from g1_navigation.launch_utils import deep_merge, merged_nav2_params, start_pose_from_areas


def test_deep_merge_overlay_wins_and_keeps_siblings():
    base = {"a": {"b": 1, "c": 2}, "d": 3}
    deep_merge(base, {"a": {"b": 10}, "e": 5})
    assert base == {"a": {"b": 10, "c": 2}, "d": 3, "e": 5}


def test_start_pose_from_areas(tmp_path):
    (tmp_path / "areas").mkdir()
    qz, qw = math.sin(-1.989 / 2), math.cos(-1.989 / 2)
    (tmp_path / "areas" / "areas_m.json").write_text(
        json.dumps({"start_location": {"safe_place": [4.0, -11.0, 0, 0, 0, qz, qw]}})
    )
    x, y, yaw = start_pose_from_areas(str(tmp_path), "m")
    assert (x, y) == (4.0, -11.0)
    assert abs(yaw + 1.989) < 1e-9
    assert start_pose_from_areas(str(tmp_path), "missing") is None


def test_merged_params_use_base_link(tmp_path):
    cfg = os.path.join(os.path.dirname(__file__), "..", "..", "g1_nav_overlay", "config")
    for mode in ("gz", "mujoco", "robot"):
        path = merged_nav2_params(cfg, mode, extra=("nav2_g1_following.yaml",), name=f"t_{mode}.yaml")
        p = yaml.safe_load(open(path))
        assert p["bt_navigator"]["ros__parameters"]["robot_base_frame"] == "base_link"
        assert p["collision_monitor"]["ros__parameters"]["base_frame_id"] == "base_link"
        sim = p["controller_server"]["ros__parameters"].get("use_sim_time", False)
        assert sim == (mode == "gz")
