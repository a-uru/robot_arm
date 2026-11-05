"""Read-only RViz preview: base-relative detected object cube."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


class Preview(Node):
    def __init__(self):
        super().__init__('aruco_preview')
        self.declare_parameter('object_size_m', .02)
        self.size = float(self.get_parameter('object_size_m').value)
        if self.size <= 0:
            raise ValueError('object_size_m must be positive')
        self.publisher = self.create_publisher(Marker, '/aruco/object_marker', 10)
        self.create_subscription(PoseStamped, '/aruco/object/pose', self.on_pose, 10)
        self.get_logger().info('Object cube uses same-image base-relative pose; no camera-to-world static TF.')

    def on_pose(self, msg):
        marker = Marker()
        marker.header = msg.header
        marker.ns = 'aruco_object'
        marker.id = 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = msg.pose
        marker.scale.x = marker.scale.y = marker.scale.z = self.size
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = .1, .8, .9, .8
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 500000000
        self.publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = Preview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
