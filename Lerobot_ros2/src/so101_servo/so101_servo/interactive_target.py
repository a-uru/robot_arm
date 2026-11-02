"""RViz draggable position target; starts at the current tool without commanding motion."""
import math
from copy import deepcopy
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PointStamped, Pose
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from .kinematics import offset_point
from tf2_ros import Buffer, TransformListener, TransformException


def make_marker(frame, position):
    marker = InteractiveMarker()
    marker.header.frame_id = frame
    marker.name = "target"
    marker.description = "Servo target: drag sphere / XYZ arrows"
    marker.scale = 0.10
    marker.pose.position = deepcopy(position)
    marker.pose.orientation.w = 1.0
    control = InteractiveMarkerControl()
    control.name = "move_3d"
    control.orientation.w = 1.0
    control.interaction_mode = InteractiveMarkerControl.MOVE_3D
    control.always_visible = True
    sphere = Marker()
    sphere.type = Marker.SPHERE
    sphere.pose.orientation.w = 1.0
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.025
    sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 0.1, 0.9, 0.3, 1.0
    control.markers.append(sphere)
    marker.controls.append(control)
    for axis in ("x", "y", "z"):
        control = InteractiveMarkerControl()
        control.name = "move_" + axis
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        control.orientation.w = 1.0
        if axis == "y":
            control.orientation.w = control.orientation.z = math.sqrt(0.5)
        elif axis == "z":
            control.orientation.w = math.sqrt(0.5)
            control.orientation.y = -math.sqrt(0.5)
        marker.controls.append(control)
    return marker


class InteractiveTarget(Node):
    def __init__(self):
        super().__init__("so101_interactive_target")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tip_frame", "gripper_link")
        for axis in "xyz":
            self.declare_parameter("tip_offset_"+axis, 0.0)
        self.offset = [self.get_parameter("tip_offset_"+axis).value for axis in "xyz"]
        if not all(math.isfinite(v) for v in self.offset):
            raise ValueError("Tool offsets must be finite")
        self.base = self.get_parameter("base_frame").value
        self.tip = self.get_parameter("tip_frame").value
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.server = InteractiveMarkerServer(self, "/so101_servo/target_marker")
        self.menu = MenuHandler()
        self.menu.insert("Stop tracking / reset to current tool", callback=self.reset)
        self.publisher = self.create_publisher(PointStamped, "/so101_servo/target_point", 10)
        self.position = None
        self.active = False
        self.initialized = False
        self.create_timer(1.0/30.0, self.tick)
        self.get_logger().info("Waiting for tool TF; dragging the marker enables position tracking")

    def current_position(self):
        tf = self.buffer.lookup_transform(self.base, self.tip, Time())
        stamp = Time.from_msg(tf.header.stamp)
        if stamp.nanoseconds == 0 or (self.get_clock().now()-stamp).nanoseconds/1e9 > 0.2:
            raise ValueError("Tool TF is not fresh")
        p, q = tf.transform.translation, tf.transform.rotation
        point = offset_point([p.x, p.y, p.z], [q.x, q.y, q.z, q.w], self.offset)
        from geometry_msgs.msg import Point
        return Point(x=float(point[0]), y=float(point[1]), z=float(point[2]))

    def reset(self, feedback=None):
        # Stop target publication even when TF is unavailable.
        self.active = False
        try:
            p = self.current_position()
            from geometry_msgs.msg import Point
            self.position = Point(x=p.x, y=p.y, z=p.z)
            marker = make_marker(self.base, self.position)
            self.server.insert(marker, feedback_callback=self.feedback)
            self.menu.apply(self.server, marker.name)
            self.server.applyChanges()
            self.initialized = True
            self.get_logger().info("Marker at current tool; tracking disabled until the marker is dragged")
        except (TransformException, ValueError):
            self.get_logger().warning("Tracking disabled; waiting for fresh tool TF to reset marker")
            self.initialized = False

    def feedback(self, feedback):
        if feedback.marker_name != "target" or feedback.event_type not in (
                InteractiveMarkerFeedback.POSE_UPDATE, InteractiveMarkerFeedback.MOUSE_UP):
            return
        if feedback.header.frame_id != self.base:
            return
        p = feedback.pose.position
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            return
        # Mouse release keeps the selected target active until reset or node shutdown.
        self.position = deepcopy(p)
        self.active = True
        pose = Pose()
        pose.position = deepcopy(p)
        pose.orientation.w = 1.0
        self.server.setPose("target", pose)
        self.server.applyChanges()

    def tick(self):
        if not self.initialized:
            try:
                self.current_position()
            except (TransformException, ValueError):
                return
            self.reset()
        if not self.active or self.position is None:
            return
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base
        msg.point = deepcopy(self.position)
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = InteractiveTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
