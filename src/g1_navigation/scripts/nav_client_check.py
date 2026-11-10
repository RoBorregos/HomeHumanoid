#!/usr/bin/env python3
"""End-to-end check of the G1 navigation stack through the same services
task_manager's nav_tasks.py calls on FRIDA.

    ros2 run g1_navigation nav_client_check.py                 # read-only checks
    ros2 run g1_navigation nav_client_check.py --tour kitchen living_room exit
    ros2 run g1_navigation nav_client_check.py --tour all --laps 2 --gz-truth

Read-only checks: areas_json, get_robot_pose, get_area_for_point,
query_path, plan_patrol. --tour then calls go_to_map_area per location
(safe_place), and approach_point once if --approach. --door runs
is_door_open (blocks until the door opens or nav_central's 60 s timeout).
--gz-truth compares map -> base_link with Gazebo's model pose at each stop
(`gz model -m g1 -p`, mode gz only).

Exit code 0 only if every call succeeded.
"""

import argparse
import json
import math
import re
import subprocess
import sys
import time

import rclpy
from frida_constants.navigation_constants import (
    APPROACH_POINT_SERVICE,
    AREAS_SERVICE,
    CHECK_DOOR_SERVICE,
    GET_AREA_FOR_POINT_SERVICE,
    GET_ROBOT_POSE_SERVICE,
    MOVE_LOCATION_SERVICE,
    NAV_QUERY_SERVICE,
    PLAN_PATROL_SERVICE,
)
from frida_interfaces.srv import (
    ApproachPoint,
    CheckDoor,
    GetAreaForPoint,
    GetRobotPose,
    MapAreas,
    MoveLocation,
    NavQuery,
    PlanPatrol,
)
from geometry_msgs.msg import PointStamped
from rclpy.node import Node


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def gz_truth(model="g1"):
    """(x, y, yaw) of the Gazebo model, or None."""
    try:
        out = subprocess.run(["gz", "model", "-m", model, "-p"], capture_output=True, text=True, timeout=10).stdout
    except (OSError, subprocess.TimeoutExpired):
        return None
    nums = re.findall(r"\[([^\]]+)\]", out)
    if len(nums) < 2:
        return None
    xyz = [float(v) for v in nums[-2].split()]
    rpy = [float(v) for v in nums[-1].split()]
    return xyz[0], xyz[1], rpy[2]


class Check(Node):
    def __init__(self, timeout):
        super().__init__("nav_client_check")
        self.timeout = timeout
        self.failures = []

    def call(self, srv_type, name, request, timeout=None, label=None):
        label = label or name
        client = self.create_client(srv_type, name)
        if not client.wait_for_service(timeout_sec=10.0):
            self.fail(label, "service not available")
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout or self.timeout)
        self.destroy_client(client)
        if not future.done():
            self.fail(label, "timed out")
            return None
        return future.result()

    def fail(self, label, why):
        self.failures.append(f"{label}: {why}")
        print(f"  FAIL {label}: {why}")

    def ok(self, label, info=""):
        print(f"  ok   {label} {info}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tour", nargs="*", default=[], help="locations for go_to_map_area, or 'all'")
    ap.add_argument("--laps", type=int, default=1)
    ap.add_argument("--approach", action="store_true", help="approach_point 1.5 m ahead of the robot")
    ap.add_argument("--door", action="store_true", help="call is_door_open")
    ap.add_argument("--gz-truth", action="store_true")
    ap.add_argument("--goal-timeout", type=float, default=240.0)
    args = ap.parse_args()

    rclpy.init()
    n = Check(timeout=20.0)

    print("areas_json")
    r = n.call(MapAreas, AREAS_SERVICE, MapAreas.Request())
    areas = {}
    if r is not None:
        try:
            areas = json.loads(r.areas) if r.areas else {}
        except ValueError:
            areas = {}
        if areas:
            n.ok("areas_json", f"{len(areas)} areas: {', '.join(areas)}")
        else:
            n.fail("areas_json", "empty")

    print("get_robot_pose")
    r = n.call(GetRobotPose, GET_ROBOT_POSE_SERVICE, GetRobotPose.Request())
    pose = None
    if r is not None and r.success:
        pose = r.pose
        p = pose.pose.position
        n.ok("get_robot_pose", f"({p.x:.2f}, {p.y:.2f}, yaw {yaw_of(pose.pose.orientation):.2f}) in {pose.header.frame_id}")
    elif r is not None:
        n.fail("get_robot_pose", r.error)

    if pose is not None:
        print("get_area_for_point")
        req = GetAreaForPoint.Request()
        req.point = PointStamped()
        req.point.header.frame_id = "map"
        req.point.point = pose.pose.position
        r = n.call(GetAreaForPoint, GET_AREA_FOR_POINT_SERVICE, req)
        if r is not None and r.success:
            n.ok("get_area_for_point", f"area='{r.area}' in_house={r.in_house} near={list(r.sublocations)[:3]}")
        elif r is not None:
            n.fail("get_area_for_point", r.error)

    targets = [a for a in areas if a not in ("start_location", "start_area")]
    if len(targets) >= 2:
        print("query_path")
        req = NavQuery.Request()
        req.location_a, req.sublocation_a = targets[0], "safe_place"
        req.location_b, req.sublocation_b = targets[1], "safe_place"
        r = n.call(NavQuery, NAV_QUERY_SERVICE, req, timeout=40.0)
        if r is not None and r.success:
            n.ok("query_path", f"{targets[0]} -> {targets[1]}: {r.distance_meters:.2f} m")
        elif r is not None:
            n.fail("query_path", r.error)

    print("plan_patrol")
    req = PlanPatrol.Request()
    req.mode = "quick"
    r = n.call(PlanPatrol, PLAN_PATROL_SERVICE, req)
    if r is not None and r.success:
        n.ok("plan_patrol", f"{len(r.viewpoints)} viewpoints, {r.total_distance:.1f} m: "
             + ", ".join(f"{a}/{s}" for a, s in zip(r.areas_out, r.sublocations)))
    elif r is not None:
        n.fail("plan_patrol", r.error)

    tour = targets if args.tour == ["all"] else args.tour
    results = []
    for lap in range(args.laps):
        for loc in tour:
            print(f"go_to_map_area {loc} (lap {lap + 1})")
            req = MoveLocation.Request()
            # nav_tasks.move_to_location maps "" to "safe_place" before calling.
            req.location, req.sublocation = loc, "safe_place"
            t0 = time.time()
            r = n.call(MoveLocation, MOVE_LOCATION_SERVICE, req, timeout=args.goal_timeout, label=f"go_to {loc}")
            dt = time.time() - t0
            if r is None:
                results.append((loc, False, dt, None))
                continue
            if not r.success:
                n.fail(f"go_to {loc}", r.error)
            est = n.call(GetRobotPose, GET_ROBOT_POSE_SERVICE, GetRobotPose.Request())
            goal = areas.get(loc, {}).get("safe_place")
            info = f"{dt:.0f} s"
            err = None
            if est is not None and est.success and goal:
                p = est.pose.pose.position
                err = math.hypot(p.x - goal[0], p.y - goal[1])
                info += f", est-goal {err:.2f} m"
                if args.gz_truth:
                    gt = gz_truth()
                    if gt:
                        info += (f", est-truth {math.hypot(p.x - gt[0], p.y - gt[1]):.2f} m, "
                                 f"truth-goal {math.hypot(gt[0] - goal[0], gt[1] - goal[1]):.2f} m")
            if r.success:
                n.ok(f"go_to {loc}", info)
            results.append((loc, r.success, dt, err))

    if args.approach:
        print("approach_point")
        est = n.call(GetRobotPose, GET_ROBOT_POSE_SERVICE, GetRobotPose.Request())
        if est is not None and est.success:
            yaw = yaw_of(est.pose.pose.orientation)
            req = ApproachPoint.Request()
            req.target = PointStamped()
            req.target.header.frame_id = "map"
            req.target.point.x = est.pose.pose.position.x + 1.5 * math.cos(yaw)
            req.target.point.y = est.pose.pose.position.y + 1.5 * math.sin(yaw)
            req.target.point.z = 0.8
            req.standoff = 0.0
            r = n.call(ApproachPoint, APPROACH_POINT_SERVICE, req, timeout=args.goal_timeout)
            if r is not None and r.success:
                g = r.goal.pose.position
                n.ok("approach_point", f"goal used ({g.x:.2f}, {g.y:.2f})")
            elif r is not None:
                n.fail("approach_point", r.error)

    if args.door:
        print("is_door_open")
        r = n.call(CheckDoor, CHECK_DOOR_SERVICE, CheckDoor.Request(), timeout=90.0)
        if r is not None:
            n.ok("is_door_open", f"status={r.status}")

    if results:
        okc = sum(1 for _, s, _, _ in results if s)
        print(f"\ntour: {okc}/{len(results)} SUCCEEDED")
    print("\nFAILURES:\n  " + "\n  ".join(n.failures) if n.failures else "\nall checks passed")
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if n.failures else 0)


if __name__ == "__main__":
    main()
