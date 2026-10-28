"""Wait for valid motor feedback before activating trajectory controllers."""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def main():
    rclpy.init()
    node = Node("so101_wait_for_joint_states")
    node.declare_parameter("timeout", 30.0)
    names = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"}
    ready = False

    def on_state(msg):
        nonlocal ready
        values = dict(zip(msg.name, msg.position))
        ready = names.issubset(values) and all(math.isfinite(values[name]) for name in names)

    subscription = node.create_subscription(JointState, "/so101_follower/joint_states", on_state, 10)
    deadline = time.monotonic() + node.get_parameter("timeout").value
    try:
        while rclpy.ok() and not ready and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if ready:
            node.get_logger().info("Received all six motor positions; trajectory controllers may start.")
        else:
            node.get_logger().error("Timed out waiting for motor feedback; check the port and motor bridge.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not ready:
        raise SystemExit(1)
