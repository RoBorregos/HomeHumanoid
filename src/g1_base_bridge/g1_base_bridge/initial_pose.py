"""Publishes /initialpose once on startup, seeding slam_toolbox's localization.

Simplified port of home2's frida_gz_sim/scripts/initial_pose.py: drops the
frida_constants topic-name import (there's no team-wide convention to share
for a single package) and the subscriber-count gate tied to nav_central.py's
blocking wait (out of scope here — see docs/plan.md). Publishes repeatedly
for a few seconds since slam_toolbox's subscription may not be up yet when
this node starts (launch ordering is best-effort, not strictly sequenced).
"""

import math
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node

# Same diagonal RViz/Nav2 use for a manually-set "2D Pose Estimate".
COVARIANCE = [0.0] * 36
COVARIANCE[0] = 0.25  # x
COVARIANCE[7] = 0.25  # y
COVARIANCE[35] = 0.068  # yaw


class InitialPose(Node):
    def __init__(self):
        super().__init__("initial_pose")

        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("publish_duration", 3.0)
        self.declare_parameter("publish_period", 0.5)

        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

        x = self.get_parameter("x").value
        y = self.get_parameter("y").value
        yaw = self.get_parameter("yaw").value
        duration = self.get_parameter("publish_duration").value
        period = self.get_parameter("publish_period").value

        self._msg = PoseWithCovarianceStamped()
        self._msg.header.frame_id = "map"
        self._msg.pose.pose.position.x = x
        self._msg.pose.pose.position.y = y
        self._msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        self._msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self._msg.pose.covariance = COVARIANCE

        self._deadline = time.monotonic() + duration
        self._timer = self.create_timer(period, self._tick)
        self.get_logger().info(f"publishing /initialpose at ({x:.2f}, {y:.2f}, yaw={yaw:.2f}) for {duration}s")

    def _tick(self):
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self._msg)
        if time.monotonic() >= self._deadline:
            self.get_logger().info("done publishing /initialpose")
            self._timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = InitialPose()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
