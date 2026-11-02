"""Follow fresh PointStamped targets using position-only DLS + Servo JointJog."""
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped, Vector3Stamped
from sensor_msgs.msg import JointState
from control_msgs.msg import JointJog
from moveit_msgs.srv import ServoCommandType
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener, TransformException
from .kinematics import Chain, tracking_velocity


class PointTracker(Node):
    def __init__(self):
        super().__init__("so101_point_tracker")
        defaults = {"robot_description": "", "base_frame": "base_link", "tip_frame": "gripper_link",
                    "target_topic": "/so101_servo/target_point", "publish_rate": 50.0,
                    "target_timeout": 0.3, "state_timeout": 0.2, "transform_timeout": 0.2,
                    "gain": 4.0, "max_linear_speed": 0.08, "max_joint_speed": 0.8,
                    "max_joint_acceleration": 1.0,
                    "tip_offset_x": 0.0, "tip_offset_y": 0.0, "tip_offset_z": 0.0, "damping": 0.02, "position_tolerance": 0.003}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        for name in ("publish_rate", "target_timeout", "state_timeout", "transform_timeout", "gain",
                     "max_linear_speed", "max_joint_speed", "max_joint_acceleration", "damping", "position_tolerance"):
            if not math.isfinite(self.p[name]) or self.p[name] <= 0:
                raise ValueError(f"{name} must be finite and positive")
        self.offset = np.array([self.p["tip_offset_"+axis] for axis in "xyz"])
        if not np.all(np.isfinite(self.offset)):
            raise ValueError("Tool offsets must be finite")
        self.chain = Chain(self.p["robot_description"], self.p["base_frame"], self.p["tip_frame"])
        expected = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"}
        if set(self.chain.names) != expected:
            raise ValueError("tip_frame must be on the arm chain after wrist_roll (not on the moving jaw)")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.target = None
        self.target_received = -math.inf
        self.positions = {}
        self.state_received = -math.inf
        self.last_velocity = np.zeros(len(self.chain.names))
        self.last_tick = time.monotonic()
        self.mode_ready = False
        self.mode_future = None
        self.status_value = None
        self.command = self.create_publisher(JointJog, "/servo_node/delta_joint_cmds", 10)
        self.control_point_pub = self.create_publisher(Marker, "/so101_servo/control_point", 10)
        self.error_pub = self.create_publisher(Vector3Stamped, "/so101_servo/position_error", 10)
        self.status = self.create_publisher(String, "/so101_servo/tracker_status", 10)
        self.mode = self.create_client(ServoCommandType, "/servo_node/switch_command_type")
        self.create_subscription(PointStamped, self.p["target_topic"], self.on_target, 10)
        self.create_subscription(JointState, "/joint_states", self.on_state, qos_profile_sensor_data)
        self.create_timer(1.0/self.p["publish_rate"], self.tick)
        self.get_logger().info(f"Waiting for fresh PointStamped on {self.p['target_topic']}; no target means stop")

    def set_status(self, value):
        self.status.publish(String(data=value))
        if value != self.status_value:
            self.get_logger().info(value)
            self.status_value = value

    def stamp_valid(self, stamp, timeout):
        if stamp.sec == 0 and stamp.nanosec == 0:
            return True  # Zero stamps explicitly mean latest/receipt-time based targets.
        age = (self.get_clock().now()-Time.from_msg(stamp)).nanoseconds/1e9
        return -0.05 <= age <= timeout

    def on_target(self, msg):
        if not msg.header.frame_id or not all(math.isfinite(x) for x in (msg.point.x, msg.point.y, msg.point.z)) or not self.stamp_valid(msg.header.stamp, self.p["target_timeout"]):
            self.target = None
            self.set_status("invalid_target")
            return
        self.target = msg
        self.target_received = time.monotonic()

    def on_state(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(n in values and math.isfinite(values[n]) for n in self.chain.names) and self.stamp_valid(msg.header.stamp, self.p["state_timeout"]):
            self.positions = values
            self.state_received = time.monotonic()

    def publish(self, velocity):
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.p["base_frame"]
        msg.joint_names = self.chain.names
        msg.velocities = list(map(float, velocity))
        msg.duration = 1.0/self.p["publish_rate"]
        self.command.publish(msg)

    def stop(self, reason):
        self.last_velocity[:] = 0.0
        if self.mode_ready:
            self.publish(self.last_velocity)
        self.set_status(reason)

    def target_in_base(self):
        msg = self.target
        point = np.array([msg.point.x, msg.point.y, msg.point.z])
        if msg.header.frame_id == self.p["base_frame"]:
            return point
        tf = self.buffer.lookup_transform(self.p["base_frame"], msg.header.frame_id, Time.from_msg(msg.header.stamp), timeout=Duration(seconds=0.0))
        # Reject old dynamic transforms; static transforms have a zero stamp.
        if not self.stamp_valid(tf.header.stamp, self.p["transform_timeout"]):
            raise ValueError("Stale target transform")
        q = tf.transform.rotation
        a = np.array([q.x, q.y, q.z]); w = q.w
        norm = float(np.dot(a, a)+w*w)
        if not math.isfinite(norm) or abs(norm-1.0) > 0.01:
            raise ValueError("Invalid transform quaternion")
        point = point + 2*np.cross(a, np.cross(a, point)+w*point)
        t = tf.transform.translation
        return point + np.array([t.x, t.y, t.z])

    def tick(self):
        now = time.monotonic()
        dt = min(now-self.last_tick, 2.0/self.p["publish_rate"])
        self.last_tick = now
        if not self.mode_ready:
            if self.mode_future is not None and self.mode_future.done():
                result = self.mode_future.result()
                self.mode_ready = result is not None and result.success
                self.mode_future = None
            if not self.mode_ready:
                if self.mode_future is None and self.mode.service_is_ready():
                    self.mode_future = self.mode.call_async(ServoCommandType.Request(command_type=ServoCommandType.Request.JOINT_JOG))
                self.set_status("waiting_for_servo")
                return
        if now-self.state_received > self.p["state_timeout"]:
            self.stop("stale_joint_state")
            return
        point, _ = self.chain.evaluate(self.positions, self.offset)
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.p["base_frame"]
        marker.ns = "control_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.pose.orientation.w = 1.0
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, point)
        marker.scale.x = marker.scale.y = marker.scale.z = 0.012
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 0.5, 1.0, 1.0
        marker.lifetime.sec = 1
        self.control_point_pub.publish(marker)

        if self.target is None or now-self.target_received > self.p["target_timeout"]:
            self.stop("waiting_for_target")
            return
        try:
            target = self.target_in_base()
            point, jacobian = self.chain.evaluate(self.positions, self.offset)
            error = target-point
            msg = Vector3Stamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.p["base_frame"]
            msg.vector.x, msg.vector.y, msg.vector.z = map(float, error)
            self.error_pub.publish(msg)
            if np.linalg.norm(error) <= self.p["position_tolerance"]:
                desired = np.zeros(len(self.chain.names))
                status = "at_target"
            else:
                # DLS remains bounded at rank-deficient poses. Do not halt reachable
                # tangential motion just because a different Cartesian direction is lost.
                near_singularity = np.linalg.svd(jacobian, compute_uv=False)[-1] < 1e-5
                desired = tracking_velocity(error, jacobian, self.p["gain"], self.p["max_linear_speed"], self.p["damping"], self.p["max_joint_speed"])
                if near_singularity and np.linalg.norm(desired) < 1e-6:
                    desired[:] = 0.0
                    status = "unreachable_direction"
                else:
                    status = "tracking_near_singularity" if near_singularity else "tracking"
            step = self.p["max_joint_acceleration"]*dt
            self.last_velocity += np.clip(desired-self.last_velocity, -step, step)
            self.publish(self.last_velocity)
            self.set_status(status)
        except (TransformException, ValueError, np.linalg.LinAlgError) as exc:
            self.stop("target_transform_or_kinematics_unavailable")


def main():
    rclpy.init()
    node = PointTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop("shutdown")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
