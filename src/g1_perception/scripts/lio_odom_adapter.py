#!/usr/bin/env python3
"""FAST-LIO /Odometry (pose of its IMU "body" frame) -> /odom + odom -> base_link.

FAST-LIO estimates the pose of its body frame (the MID-360's IMU, here the
same frame as mid360_link) relative to "camera_init", which is that same frame
at startup. Its own TF (camera_init -> body) is remapped away in lio.launch.py,
because Nav2 and slam_toolbox need odom -> base_link.

With B = base_link -> mid360_link from the URDF (TF, looked up once):
    odom -> base_link(t) = B * L(t) * B^-1
where L(t) is FAST-LIO's camera_init -> body. odom then coincides with
base_link at startup, which is upright, so odom's z is up. The result is
projected to the plane (x, y, yaw), same convention as g1_leg_odom: walking
sway in roll/pitch would only tilt the costmaps' view.

initial_x/y/yaw seed odom at a known pose (sim spawn), like the bridges.
FAST-LIO leaves the twist empty, so the twist is a low-passed finite
difference of the planar pose, in base_link (controller_server feeds it to
RPP's angular ramp and to MPPI's initial state).
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


def quat_to_rot(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def make_tf(rot, trans):
    t = np.eye(4)
    t[:3, :3] = rot
    t[:3, 3] = trans
    return t


class LioOdomAdapter(Node):
    def __init__(self):
        super().__init__("lio_odom_adapter")
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.lidar_frame = self.declare_parameter("lidar_frame", "mid360_link").value
        self.publish_tf = self.declare_parameter("publish_tf", True).value
        in_topic = self.declare_parameter("input_topic", "/Odometry").value
        out_topic = self.declare_parameter("output_topic", "/odom").value
        x0 = self.declare_parameter("initial_x", 0.0).value
        y0 = self.declare_parameter("initial_y", 0.0).value
        yaw0 = self.declare_parameter("initial_yaw", 0.0).value
        self.seed = make_tf(
            np.array([[math.cos(yaw0), -math.sin(yaw0), 0], [math.sin(yaw0), math.cos(yaw0), 0], [0, 0, 1]]),
            [x0, y0, 0.0],
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, out_topic, 20)
        self.create_subscription(Odometry, in_topic, self.on_odom, 20)
        self.tau = self.declare_parameter("velocity_filter_tau", 0.1).value
        self.B = None
        self.B_inv = None
        self.warned = False
        self.prev = None  # (t, x, y, yaw)
        self.vel = np.zeros(3)  # vx, vy (odom), wz

    def lookup_extrinsic(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, self.lidar_frame, Time(), timeout=Duration(seconds=0.0)
            )
        except Exception as e:  # noqa: BLE001 - any TF failure just means "not yet"
            if not self.warned:
                self.get_logger().warn(f"waiting for TF {self.base_frame} -> {self.lidar_frame}: {e}")
                self.warned = True
            return False
        r = t.transform.rotation
        p = t.transform.translation
        self.B = make_tf(quat_to_rot(r.x, r.y, r.z, r.w), [p.x, p.y, p.z])
        self.B_inv = np.linalg.inv(self.B)
        self.get_logger().info(
            f"extrinsic {self.base_frame} -> {self.lidar_frame}: t=({p.x:.3f}, {p.y:.3f}, {p.z:.3f})"
        )
        return True

    def on_odom(self, msg: Odometry):
        if self.B is None and not self.lookup_extrinsic():
            return
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        L = make_tf(quat_to_rot(q.x, q.y, q.z, q.w), [p.x, p.y, p.z])
        T = self.seed @ self.B @ L @ self.B_inv
        yaw = math.atan2(T[1, 0], T[0, 0])
        x, y = T[0, 3], T[1, 3]

        t_now = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if self.prev is not None:
            dt = t_now - self.prev[0]
            if 1e-4 < dt < 1.0:
                dyaw = math.atan2(math.sin(yaw - self.prev[3]), math.cos(yaw - self.prev[3]))
                raw = np.array([(x - self.prev[1]) / dt, (y - self.prev[2]) / dt, dyaw / dt])
                a = dt / (self.tau + dt)
                self.vel = (1 - a) * self.vel + a * raw
        self.prev = (t_now, x, y, yaw)
        c, s = math.cos(yaw), math.sin(yaw)

        o = Odometry()
        o.header.stamp = msg.header.stamp
        o.header.frame_id = self.odom_frame
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x = x
        o.pose.pose.position.y = y
        o.pose.pose.orientation.z = math.sin(yaw / 2)
        o.pose.pose.orientation.w = math.cos(yaw / 2)
        o.twist.twist.linear.x = c * self.vel[0] + s * self.vel[1]
        o.twist.twist.linear.y = -s * self.vel[0] + c * self.vel[1]
        o.twist.twist.angular.z = float(self.vel[2])
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.001
        cov[35] = 0.001
        cov[14] = cov[21] = cov[28] = 1e3
        o.pose.covariance = cov
        o.twist.covariance = cov
        self.pub.publish(o)

        if self.publish_tf:
            t = TransformStamped()
            t.header = o.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.rotation = o.pose.pose.orientation
            self.tf_pub.sendTransform(t)


def main():
    rclpy.init()
    node = LioOdomAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
