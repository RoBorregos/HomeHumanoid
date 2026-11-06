#!/usr/bin/env python3
"""Lowers the MuJoCo elastic band until the G1's feet carry its weight, then releases it.

Sim only. The band is driven by keystrokes to the MuJoCo window, so this needs xdotool
and DISPLAY set. Run it once g1_rl_bridge reports it is accepting cmd_vel: releasing
before the policy is walking drops the robot. Height comes from /odom's z.
"""
import subprocess
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

SETTLE_S = 5.0
MIN_STANDING_Z = 0.7


def main():
    rclpy.init()
    node = Node("g1_release_band")
    state = {}
    node.create_subscription(Odometry, "/odom", lambda m: state.update(z=m.pose.pose.position.z), 10)

    def spin(seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    window = subprocess.run(
        ["xdotool", "search", "--name", "MuJoCo"], capture_output=True, text=True, check=True
    ).stdout.split()[-1]

    def press(key):
        subprocess.run(["xdotool", "windowactivate", "--sync", window, "key", key], check=True)

    deadline = time.time() + 20.0
    while "z" not in state:
        if time.time() > deadline:
            raise SystemExit("no /odom received, is g1_rl_bridge running?")
        spin(0.1)
    spin(2.0)
    last, flat = state["z"], 0
    for _ in range(30):
        press("8")
        spin(1.5)
        flat = flat + 1 if abs(state["z"] - last) < 0.01 else 0
        last = state["z"]
        if flat >= 2:
            break
    spin(SETTLE_S)
    print(f"lowered, z={state['z']:.3f}")
    press("9")
    spin(SETTLE_S)
    print(f"released, z={state['z']:.3f}")
    ok = state["z"] > MIN_STANDING_Z
    rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
