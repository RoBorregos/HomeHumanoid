"""Bridge between Nav2's cmd_vel and the G1's active locomotion backend.

Track A (default): republishes straight to the command topic of Gazebo's
VelocityControl plugin, same as FRIDA today (see docs/plan.md, M2a).

Track B: translates to unitree_rl_lab's command interface. The exact format
("/command" std_msgs/String "vel vx vy vyaw") comes from the initial research
and is NOT confirmed against the real source of
deploy/robots/g1/real_g1_bridge.py / controller_bridge.py (see open question
#1 in docs/plan.md) — do not use in M2b without verifying first.
"""

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

BACKEND_GAZEBO = "gazebo"
BACKEND_RL_POLICY = "rl_policy"


class G1BaseBridge(Node):
    def __init__(self):
        super().__init__("g1_base_bridge")

        self.declare_parameter("backend", BACKEND_GAZEBO)
        self.declare_parameter("cmd_vel_in_topic", "/cmd_vel")
        self.declare_parameter("cmd_vel_stamped", True)
        self.declare_parameter("gazebo_cmd_vel_topic", "/model/g1/cmd_vel")
        self.declare_parameter("rl_policy_command_topic", "/command")

        self.backend = self.get_parameter("backend").value
        cmd_vel_in_topic = self.get_parameter("cmd_vel_in_topic").value
        self.cmd_vel_stamped = self.get_parameter("cmd_vel_stamped").value

        msg_type = TwistStamped if self.cmd_vel_stamped else Twist
        self.create_subscription(msg_type, cmd_vel_in_topic, self._on_cmd_vel, 10)

        if self.backend == BACKEND_GAZEBO:
            gz_topic = self.get_parameter("gazebo_cmd_vel_topic").value
            self._gz_pub = self.create_publisher(Twist, gz_topic, 10)
        elif self.backend == BACKEND_RL_POLICY:
            self.get_logger().warn(
                "backend=rl_policy uses an interface NOT confirmed against "
                "unitree_rl_lab (see docs/plan.md, open question #1). "
                "Verify real_g1_bridge.py/controller_bridge.py before using "
                "it on hardware or in MuJoCo for real."
            )
            rl_topic = self.get_parameter("rl_policy_command_topic").value
            self._rl_pub = self.create_publisher(String, rl_topic, 10)
        else:
            raise ValueError(f"unknown backend: {self.backend}")

        self.get_logger().info(
            f"g1_base_bridge started (backend={self.backend}, "
            f"in={cmd_vel_in_topic}, stamped={self.cmd_vel_stamped})"
        )

    def _on_cmd_vel(self, msg):
        twist = msg.twist if self.cmd_vel_stamped else msg

        if self.backend == BACKEND_GAZEBO:
            self._gz_pub.publish(twist)
        elif self.backend == BACKEND_RL_POLICY:
            cmd = String()
            cmd.data = f"vel {twist.linear.x:.3f} {twist.linear.y:.3f} {twist.angular.z:.3f}"
            self._rl_pub.publish(cmd)


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
