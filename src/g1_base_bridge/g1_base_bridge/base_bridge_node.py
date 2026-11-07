"""Bridge between Nav2's cmd_vel and the G1's active locomotion backend.

Track A (default, backend=gazebo): pure rclpy. Republishes straight to the
command topic of Gazebo's VelocityControl plugin, same as FRIDA today (see
docs/plan.md). Includes a watchdog (ported from home2's cmd_vel_relay.py)
that zeroes the published Twist if no cmd_vel arrives within cmd_vel_timeout
seconds — Gazebo's VelocityControl plugin holds the last command forever
otherwise, so without this the robot keeps drifting after Nav2 stops talking
(lost connection, finished a goal mid-motion-blend, etc).

Also publishes /odom and broadcasts the odom -> pelvis TF itself, computed
by integrating the same cmd_vel it relays to Gazebo rather than reading
Gazebo's OdometryPublisher plugin. That plugin's own <tf_topic> output was
already confirmed empty on the Orin, but a second, worse issue killed the
plugin entirely: its /sim/odom topic published exactly once (at spawn) and
then never again, even while `gz topic -e` on the world's pose/info topic
confirmed the model kept moving under VelocityControl — i.e. the plugin
isn't just missing TF, it stops publishing odometry at all shortly after
startup on this Gazebo Harmonic build. Since Track A's base is velocity-
controlled (not physically simulated drift/slip), dead-reckoning from the
commanded vx/vy/wz is exact — identical to what the plugin would report if
it worked — so this sidesteps the plugin completely instead of chasing it
further.

Track A only. Track B (real walking in MuJoCo) lives in the g1_rl_bridge package.

A repeated cmd_vel is not a live one: during one run collision_monitor kept
republishing the same command at 20Hz with nothing feeding it, so an
arrival-time watchdog alone never fired and the robot drove off the map.
Messages whose stamp is older than cmd_vel_timeout are therefore dropped.
"""

import math
import threading
import time

from geometry_msgs.msg import Twist, TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformBroadcaster

BACKEND_GAZEBO = "gazebo"

class G1BaseBridge(Node):
    def __init__(self):
        super().__init__("g1_base_bridge")

        self.declare_parameter("backend", BACKEND_GAZEBO)
        self.declare_parameter("cmd_vel_in_topic", "/cmd_vel")
        self.declare_parameter("cmd_vel_stamped", True)
        self.declare_parameter("gazebo_cmd_vel_topic", "/sim/cmd_vel")
        self.declare_parameter("cmd_vel_timeout", 0.5)
        self.declare_parameter("odom_out_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "pelvis")
        self.declare_parameter("odom_publish_frequency", 50.0)
        self.declare_parameter("spawn_x", 0.0)
        self.declare_parameter("spawn_y", 0.0)
        self.declare_parameter("spawn_yaw", 0.0)

        self.backend = self.get_parameter("backend").value
        self._cmd_vel_timeout = self.get_parameter("cmd_vel_timeout").value
        cmd_vel_in_topic = self.get_parameter("cmd_vel_in_topic").value
        self.cmd_vel_stamped = self.get_parameter("cmd_vel_stamped").value

        msg_type = TwistStamped if self.cmd_vel_stamped else Twist
        self.create_subscription(msg_type, cmd_vel_in_topic, self._on_cmd_vel, 10)

        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._lock = threading.Lock()

        if self.backend == BACKEND_GAZEBO:
            gz_topic = self.get_parameter("gazebo_cmd_vel_topic").value
            self._gz_pub = self.create_publisher(Twist, gz_topic, 10)
            self._last_cmd_vel_time = 0.0
            self._gz_stopped = True
            self.create_timer(self._cmd_vel_timeout / 2.0, self._gazebo_watchdog)

            self._odom_frame = self.get_parameter("odom_frame").value
            self._base_frame = self.get_parameter("base_frame").value
            self._odom_x = self.get_parameter("spawn_x").value
            self._odom_y = self.get_parameter("spawn_y").value
            self._odom_yaw = self.get_parameter("spawn_yaw").value
            self._odom_pub = self.create_publisher(
                Odometry, self.get_parameter("odom_out_topic").value, 10
            )
            self._tf_broadcaster = TransformBroadcaster(self)
            odom_freq = self.get_parameter("odom_publish_frequency").value
            self._last_odom_time = self.get_clock().now()
            self.create_timer(1.0 / odom_freq, self._publish_odom)
        else:
            raise ValueError(f"unknown backend: {self.backend}")

        self.get_logger().info(
            f"g1_base_bridge started (backend={self.backend}, in={cmd_vel_in_topic}, "
            f"stamped={self.cmd_vel_stamped})"
        )

    def _on_cmd_vel(self, msg):
        twist = msg.twist if self.cmd_vel_stamped else msg
        if self.cmd_vel_stamped:
            age = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
            if age > self._cmd_vel_timeout:
                self.get_logger().warning(f"Dropping stale cmd_vel, {age:.2f}s old", throttle_duration_sec=2.0)
                return
        with self._lock:
            self._vx = twist.linear.x
            self._vy = twist.linear.y
            self._wz = twist.angular.z
        if self.backend == BACKEND_GAZEBO:
            self._last_cmd_vel_time = time.monotonic()
            self._gz_stopped = False
            self._gz_pub.publish(twist)

    # ---- Track A watchdog (ported from home2's cmd_vel_relay.py) ----

    def _gazebo_watchdog(self):
        if self._gz_stopped or time.monotonic() - self._last_cmd_vel_time < self._cmd_vel_timeout:
            return
        self._gz_stopped = True
        self._gz_pub.publish(Twist())
        with self._lock:
            self._vx = self._vy = self._wz = 0.0

    # ---- Track A odometry + TF (see module docstring: Gazebo's
    # OdometryPublisher plugin stops publishing shortly after spawn on this
    # build, so this dead-reckons from the same cmd_vel already being
    # relayed to Gazebo instead) ----

    def _publish_odom(self):
        now = self.get_clock().now()
        dt = (now - self._last_odom_time).nanoseconds * 1e-9
        self._last_odom_time = now
        with self._lock:
            vx, vy, wz = self._vx, self._vy, self._wz

        self._odom_yaw += wz * dt
        self._odom_x += (vx * math.cos(self._odom_yaw) - vy * math.sin(self._odom_yaw)) * dt
        self._odom_y += (vx * math.sin(self._odom_yaw) + vy * math.cos(self._odom_yaw)) * dt
        qz = math.sin(self._odom_yaw / 2.0)
        qw = math.cos(self._odom_yaw / 2.0)
        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._odom_x
        odom.pose.pose.position.y = self._odom_y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self._odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self._odom_frame
        t.child_frame_id = self._base_frame
        t.transform.translation.x = self._odom_x
        t.transform.translation.y = self._odom_y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = G1BaseBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
