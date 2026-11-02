"""Bridge between Nav2's cmd_vel and the G1's active locomotion backend.

Track A (default, backend=gazebo): pure rclpy. Republishes straight to the
command topic of Gazebo's VelocityControl plugin, same as FRIDA today (see
docs/plan.md, M2a).

Track B (backend=rl_policy): a hybrid process. In addition to being an rclpy
node (to receive Nav2's cmd_vel), it opens a unitree_sdk2_python DDS
participant and relays the robot's real rt/lowstate: every sample received is
republished immediately, identical except for the embedded `wireless_remote`
bytes, which are overridden with the current cmd_vel packed as joystick
axes. This is the mechanism unitree_rl_lab's FSM actually reads (there is no
separate WirelessController_ topic and no "/command" string topic for G1 —
see docs/plan.md, "M1 findings", confirmed on the Orin against the real
unitree_sdk2/unitree_rl_lab sources).

On startup, Track B also drives the one-time button sequence
(Passive -> FixStand -> Velocity) the same way, since there is no lower-level
entry point that bypasses unitree_rl_lab's FSM.

NOTE: the wireless_remote byte layout, bit positions, and the relay pattern
itself were validated end-to-end on the Orin using unitree_sdk2's C++ API
(see third_party/lowstate_relay.cc, third_party/walk_relay.cc — not vendored
here, see third_party/README.md). This file uses unitree_sdk2_python instead
(needed to share a process/event loop with rclpy) — the exact import paths
below follow unitree_sdk2_python's documented ChannelFactoryInitialize/
ChannelPublisher/ChannelSubscriber pattern but have NOT yet been run; verify
against the installed unitree_sdk2_python version before first use.
"""

import struct
import threading
import time

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node

BACKEND_GAZEBO = "gazebo"
BACKEND_RL_POLICY = "rl_policy"

# Bit positions in the 16-bit button mask at wireless_remote bytes 2-3.
# Source: unitree_sdk2/include/unitree/dds_wrapper/common/unitree_joystick.hpp
BTN_R1 = 1 << 0
BTN_L1 = 1 << 1
BTN_START = 1 << 2
BTN_SELECT = 1 << 3
BTN_R2 = 1 << 4
BTN_L2 = 1 << 5
BTN_F1 = 1 << 6
BTN_F2 = 1 << 7
BTN_A = 1 << 8
BTN_B = 1 << 9
BTN_X = 1 << 10
BTN_Y = 1 << 11
BTN_UP = 1 << 12
BTN_RIGHT = 1 << 13
BTN_DOWN = 1 << 14
BTN_LEFT = 1 << 15

# Byte offsets into the 40-byte wireless_remote blob (REMOTE_DATA_RX):
# head[2], btn(u16), lx(f32), rx(f32), ry(f32), L2(f32, unused), ly(f32).
OFFSET_BTN = 2
OFFSET_LX = 4
OFFSET_RX = 8
OFFSET_RY = 12
OFFSET_LY = 20


def _pack_wireless_remote(keys: int, lx: float = 0.0, ly: float = 0.0, rx: float = 0.0) -> bytes:
    buf = bytearray(40)
    struct.pack_into("<H", buf, OFFSET_BTN, keys)
    struct.pack_into("<f", buf, OFFSET_LX, lx)
    struct.pack_into("<f", buf, OFFSET_RX, rx)
    struct.pack_into("<f", buf, OFFSET_LY, ly)
    return bytes(buf)


class G1BaseBridge(Node):
    def __init__(self):
        super().__init__("g1_base_bridge")

        self.declare_parameter("backend", BACKEND_GAZEBO)
        self.declare_parameter("cmd_vel_in_topic", "/cmd_vel")
        self.declare_parameter("cmd_vel_stamped", True)
        self.declare_parameter("gazebo_cmd_vel_topic", "/model/g1/cmd_vel")
        self.declare_parameter("dds_domain_id", 0)
        self.declare_parameter("dds_network_interface", "lo")
        self.declare_parameter("cmd_vel_axis_scale", 1.0)

        self.backend = self.get_parameter("backend").value
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
        elif self.backend == BACKEND_RL_POLICY:
            self._start_rl_policy_backend()
        else:
            raise ValueError(f"unknown backend: {self.backend}")

        self.get_logger().info(
            f"g1_base_bridge started (backend={self.backend}, in={cmd_vel_in_topic}, "
            f"stamped={self.cmd_vel_stamped})"
        )

    def _on_cmd_vel(self, msg):
        twist = msg.twist if self.cmd_vel_stamped else msg
        if self.backend == BACKEND_GAZEBO:
            self._gz_pub.publish(twist)
        elif self.backend == BACKEND_RL_POLICY:
            with self._lock:
                self._vx = twist.linear.x
                self._vy = twist.linear.y
                self._wz = twist.angular.z

    # ---- Track B: rt/lowstate relay-with-override (see module docstring) ----

    def _start_rl_policy_backend(self):
        # Deferred import: unitree_sdk2_python is a pip package, not a rosdep
        # key, and is only needed for this backend.
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize,
            ChannelPublisher,
            ChannelSubscriber,
        )
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

        domain_id = self.get_parameter("dds_domain_id").value
        iface = self.get_parameter("dds_network_interface").value
        self._axis_scale = self.get_parameter("cmd_vel_axis_scale").value

        ChannelFactoryInitialize(domain_id, iface)
        self._lowstate_pub = ChannelPublisher("rt/lowstate", LowState_)
        self._lowstate_pub.Init()

        self._startup_sequence_done = threading.Event()
        threading.Thread(target=self._run_startup_sequence, daemon=True).start()

        self._lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self._lowstate_sub.Init(self._on_lowstate, 10)

    def _run_startup_sequence(self):
        """One-time Passive -> FixStand -> Velocity transition.

        Drives the FSM via the same wireless_remote bits as a real gamepad
        (see docs/plan.md, "M1 findings"). The relay in _on_lowstate() keeps
        running throughout and simply forwards whatever _startup_keys holds
        until this sequence releases it, then switches to cmd_vel axes.
        """
        self._startup_keys = 0
        time.sleep(0.5)
        self._startup_keys = BTN_L2
        time.sleep(0.5)
        self._startup_keys = BTN_L2 | BTN_UP
        time.sleep(0.3)
        self._startup_keys = 0
        time.sleep(0.8)
        self._startup_keys = BTN_R1
        time.sleep(0.5)
        self._startup_keys = BTN_R1 | BTN_X
        time.sleep(0.3)
        self._startup_keys = 0
        self._startup_sequence_done.set()

    def _on_lowstate(self, msg):
        keys = 0 if self._startup_sequence_done.is_set() else getattr(self, "_startup_keys", 0)
        with self._lock:
            vx, vy, wz = self._vx, self._vy, self._wz
        lx = max(-1.0, min(1.0, vx * self._axis_scale))
        ly = max(-1.0, min(1.0, vy * self._axis_scale))
        rx = max(-1.0, min(1.0, wz * self._axis_scale))
        msg.wireless_remote = _pack_wireless_remote(keys, lx=lx, ly=ly, rx=rx)
        self._lowstate_pub.Write(msg)


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
