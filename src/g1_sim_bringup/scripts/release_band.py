#!/usr/bin/env python3
"""Lowers the MuJoCo elastic band until the G1's feet carry its weight, then releases it.

Sim only. The band is driven by keystrokes to the MuJoCo window, so this needs xdotool
and DISPLAY set. Run it once g1_rl_bridge reports it is accepting cmd_vel: releasing
before the policy is walking drops the robot. Height comes from MuJoCo's ground truth,
on /odom_ground_truth with odom_source:=leg (the default) or /odom otherwise.
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
    heights = {}
    # Ground truth wins when present: g1_leg_odom's /odom z is height over the foot,
    # which doesn't drop when the band lowers the robot.
    for topic in ("/odom_ground_truth", "/odom"):
        node.create_subscription(
            Odometry, topic, lambda m, t=topic: heights.__setitem__(t, m.pose.pose.position.z), 10)

    def update_z():
        for topic in ("/odom_ground_truth", "/odom"):
            if topic in heights:
                state["z"] = heights[topic]
                return

    def spin(seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
            update_z()

    window = subprocess.run(
        ["xdotool", "search", "--name", "MuJoCo"], capture_output=True, text=True, check=True
    ).stdout.split()[-1]

    def press(key):
        # windowactivate needs a window manager; a bare Xvfb has none, so fall
        # back to setting the input focus directly.
        if subprocess.run(["xdotool", "windowactivate", "--sync", window], capture_output=True).returncode:
            subprocess.run(["xdotool", "windowfocus", "--sync", window], check=True)
        subprocess.run(["xdotool", "key", key], check=True)

    deadline = time.time() + 20.0
    while "z" not in state:
        if time.time() > deadline:
            raise SystemExit("no /odom_ground_truth or /odom received, is g1_rl_bridge running?")
        spin(0.1)
    spin(2.0)
    print(f"height from {next(t for t in ('/odom_ground_truth', '/odom') if t in heights)}")
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
