#!/usr/bin/env python3
"""Compares /odom (g1_leg_odom) with MuJoCo's ground truth on /odom_ground_truth.

Sim only, with sim_mujoco.launch.py odom_source:=leg. Both topics share the odom
frame and the spawn pose, so their difference is the estimator's drift. Prints
one line every --period seconds and a summary on Ctrl-C, optionally logging every
sample pair to a CSV.

    python3 odom_error.py --period 5 --csv /tmp/g1nav/odom_error.csv
"""

import argparse
import csv
import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


def yaw_of(m):
    q = m.pose.pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class OdomError(Node):
    def __init__(self, period, csv_path):
        super().__init__("g1_odom_error")
        self.est = None
        self.gt_prev = None
        self.dist = 0.0
        self.max_err = 0.0
        self.last = None
        self.writer = None
        if csv_path:
            self.csv_file = open(csv_path, "w", newline="")
            self.writer = csv.writer(self.csv_file)
            self.writer.writerow(
                [
                    "t",
                    "gt_x",
                    "gt_y",
                    "gt_yaw",
                    "est_x",
                    "est_y",
                    "est_yaw",
                    "err_xy",
                    "err_yaw",
                ]
            )
        self.create_subscription(Odometry, "/odom", self.on_est, 10)
        self.create_subscription(Odometry, "/odom_ground_truth", self.on_gt, 10)
        self.create_timer(period, self.report)

    def on_est(self, m):
        self.est = m

    def on_gt(self, m):
        if self.est is None:
            return
        g, e = m.pose.pose.position, self.est.pose.pose.position
        if self.gt_prev is not None:
            self.dist += math.hypot(g.x - self.gt_prev[0], g.y - self.gt_prev[1])
        self.gt_prev = (g.x, g.y)
        err = math.hypot(e.x - g.x, e.y - g.y)
        err_yaw = wrap(yaw_of(self.est) - yaw_of(m))
        self.max_err = max(self.max_err, err)
        self.last = (err, err_yaw, g, e)
        if self.writer:
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            self.writer.writerow(
                [
                    f"{t:.3f}",
                    g.x,
                    g.y,
                    yaw_of(m),
                    e.x,
                    e.y,
                    yaw_of(self.est),
                    err,
                    err_yaw,
                ]
            )

    def report(self):
        if self.last is None:
            self.get_logger().info("waiting for /odom and /odom_ground_truth")
            return
        err, err_yaw, g, e = self.last
        pct = 100 * err / self.dist if self.dist > 0.5 else float("nan")
        self.get_logger().info(
            f"walked {self.dist:6.2f} m | err {err:5.3f} m ({pct:4.1f}% of distance), "
            f"yaw {math.degrees(err_yaw):+5.1f} deg, max {self.max_err:5.3f} m | "
            f"gt ({g.x:.2f}, {g.y:.2f}) est ({e.x:.2f}, {e.y:.2f})"
        )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--period", type=float, default=5.0)
    ap.add_argument("--csv", default="")
    args = ap.parse_args()
    rclpy.init()
    node = OdomError(args.period, args.csv)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.report()
    if node.writer:
        node.csv_file.close()


if __name__ == "__main__":
    main()
