import time
import cv2
import numpy as np
import rclpy
import message_filters
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from .transforms import from_xyz_rpy, from_pose, base_object, camera_link_to_optical
from .vision import detect, dictionary, estimate


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        defaults = {'dictionary': 'DICT_4X4_50', 'marker_size': 0.02,
                    'marker_ids': [0, 1], 'marker_sizes': [0.02, 0.02],
                    'image_topic': '/image_raw', 'camera_info_topic': '/camera_info',
                    'max_reprojection_error': 3.0,
                    'pose_ambiguity_px': 0.3, 'pose_history_timeout_s': 0.5,
                    'pose_refine_lm': True, 'corner_refinement_window': 5,
                    'corner_refinement_iterations': 50, 'corner_refinement_accuracy': 0.001,
                    'base_prediction_frame': 'base_link_predict',
                    'reference_id': 0, 'object_id': 1, 'base_frame': 'base_link',
                    'object_frame': 'aruco_object_origin', 'base_offset_calibrated': False,
                    'publish_camera_tf': True, 'camera_link_frame': 'camera_link',
                    'base_to_reference.xyz_m': [0.0, 0.0, 0.0],
                    'base_to_reference.rpy_deg': [0.0, 0.0, 0.0],
                    'marker_to_object.xyz_m': [0.0, 0.0, 0.0],
                    'marker_to_object.rpy_deg': [0.0, 0.0, 0.0]}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        value = lambda key: self.get_parameter(key).value
        self.reference_id = int(value('reference_id'))
        self.object_id = int(value('object_id'))
        if self.reference_id == self.object_id:
            raise ValueError('reference_id and object_id must be different')
        self.base_prediction_frame = value('base_prediction_frame')
        self.base_frame = value('base_frame')
        self.object_frame = value('object_frame')
        if not self.base_frame or not self.object_frame or self.base_frame == self.object_frame:
            raise ValueError('Distinct nonempty base_frame and object_frame are required')
        self.offset_valid = value('base_offset_calibrated')
        self.publish_camera_tf = value('publish_camera_tf')
        self.camera_link_frame = value('camera_link_frame')
        if not self.camera_link_frame or self.camera_link_frame == self.base_frame:
            raise ValueError('camera_link_frame must be nonempty and differ from base_frame')
        self.static_optical_frame = None
        self.base_reference = from_xyz_rpy(value('base_to_reference.xyz_m'), value('base_to_reference.rpy_deg'))
        self.marker_object = from_xyz_rpy(value('marker_to_object.xyz_m'), value('marker_to_object.rpy_deg'))
        self.object_pose = self.create_publisher(PoseStamped, '/aruco/object/pose', 10)
        self.marker_dictionary = dictionary(value('dictionary'))
        self.size = float(value('marker_size'))
        ids, sizes = value('marker_ids'), value('marker_sizes')
        if len(ids) != len(sizes) or self.size <= 0 or any(s <= 0 for s in sizes):
            raise ValueError('marker_ids/marker_sizes must match; sizes must be positive metres')
        self.sizes = dict(zip(ids, sizes))
        self.max_error = float(value('max_reprojection_error'))
        self.refine_lm = bool(value('pose_refine_lm'))
        self.corner_window = int(value('corner_refinement_window'))
        self.corner_iterations = int(value('corner_refinement_iterations'))
        self.corner_accuracy = float(value('corner_refinement_accuracy'))
        if self.corner_window < 1 or self.corner_iterations < 1 or self.corner_accuracy <= 0:
            raise ValueError('Corner refinement parameters must be positive')
        self.ambiguity_px = float(value('pose_ambiguity_px'))
        self.history_timeout = float(value('pose_history_timeout_s'))
        if self.ambiguity_px < 0 or self.history_timeout < 0:
            raise ValueError('Pose ambiguity and timeout must be nonnegative')
        self.pose_history = {}
        self.bridge = CvBridge()
        self.info = None
        self.publishers_by_id = {}
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.debug = self.create_publisher(Image, '/aruco/image', 1)
        self.ids = self.create_publisher(Int32MultiArray, '/aruco/detected_ids', 10)
        self.image_sub = message_filters.Subscriber(self, Image, value('image_topic'), qos_profile=qos_profile_sensor_data)
        self.info_sub = message_filters.Subscriber(self, CameraInfo, value('camera_info_topic'), qos_profile=qos_profile_sensor_data)
        self.synchronizer = message_filters.TimeSynchronizer([self.image_sub, self.info_sub], 10)
        self.synchronizer.registerCallback(self.on_synchronized)
        self.get_logger().info('ArUco ready. Pose requires calibrated CameraInfo; output is camera optical coordinates.')

    def on_synchronized(self, image, info):
        self.on_info(info)
        self.on_image(image)

    def on_info(self, msg):
        self.info = msg

    def on_image(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Image conversion failed: {exc}', throttle_duration_sec=5.0)
            return
        poses = {}
        corners, ids, _ = detect(image, self.marker_dictionary, self.corner_window,
                                  self.corner_iterations, self.corner_accuracy)
        self.ids.publish(Int32MultiArray(data=[] if ids is None else ids.flatten().tolist()))
        info = self.info
        reason = None
        if info is None or info.k[0] <= 0 or info.k[4] <= 0:
            reason = 'No calibrated CameraInfo: detection only, no 3D poses'
        elif info.width != msg.width or info.height != msg.height:
            reason = 'CameraInfo/image size mismatch: no 3D poses'
        elif not msg.header.frame_id or info.header.frame_id != msg.header.frame_id:
            reason = 'Image/CameraInfo frame_id missing or different: no 3D poses'
        elif info.distortion_model not in ('plumb_bob', 'rational_polynomial'):
            reason = f'Unsupported distortion model: {info.distortion_model}'
        elif info.roi.width or info.roi.height or info.binning_x > 1 or info.binning_y > 1:
            reason = 'ROI/binning CameraInfo is unsupported; use full-resolution raw images'
        if reason:
            self.get_logger().warning(reason, throttle_duration_sec=5.0)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(image, corners, ids)
            if reason is None:
                matrix = np.array(info.k, dtype=float).reshape(3, 3)
                distortion = np.array(info.d, dtype=float)
                for marker_corners, marker_id in zip(corners, ids.flatten()):
                    marker_id = int(marker_id)
                    size = self.sizes.get(marker_id, self.size)
                    now = time.monotonic()
                    history = self.pose_history.get(marker_id)
                    previous = history[1] if history and now - history[0] <= self.history_timeout else None
                    result = estimate(marker_corners, size, matrix, distortion, previous, self.ambiguity_px, self.refine_lm)
                    if result is None:
                        continue
                    rotation, translation, error = result
                    if error > self.max_error:
                        continue
                    self.pose_history[marker_id] = (now, rotation.copy())
                    quaternion = Rotation.from_rotvec(rotation.flatten()).as_quat()
                    pose = PoseStamped()
                    pose.header = msg.header
                    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = translation.flatten().tolist()
                    (pose.pose.orientation.x, pose.pose.orientation.y,
                     pose.pose.orientation.z, pose.pose.orientation.w) = quaternion.tolist()
                    if marker_id not in self.publishers_by_id:
                        self.publishers_by_id[marker_id] = self.create_publisher(
                            PoseStamped, f'/aruco/marker_{marker_id}/pose', 10)
                    self.publishers_by_id[marker_id].publish(pose)
                    poses[marker_id] = from_pose(pose.pose)
                    transform = TransformStamped()
                    transform.header = msg.header
                    transform.child_frame_id = f'aruco_marker_{marker_id}'
                    transform.transform.translation.x = pose.pose.position.x
                    transform.transform.translation.y = pose.pose.position.y
                    transform.transform.translation.z = pose.pose.position.z
                    transform.transform.rotation = pose.pose.orientation
                    self.tf.sendTransform(transform)
                    cv2.drawFrameAxes(image, matrix, distortion, rotation, translation, size * 0.5)
                    x, y = np.asarray(marker_corners).reshape(4, 2)[0].astype(int)
                    cv2.putText(image, f'{translation[2, 0]:.3f}m err={error:.2f}px',
                                (int(x), max(15, int(y)-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        self.publish_offsets(msg.header, poses)
        if reason:
            cv2.putText(image, 'UNCALIBRATED / INVALID CAMERA INFO: detection only', (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
        debug = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        debug.header = msg.header
        self.debug.publish(debug)

    def send_matrix(self, header, parent, child, matrix, static=False):
        transform = TransformStamped()
        transform.header.stamp = header.stamp
        transform.header.frame_id = parent
        transform.child_frame_id = child
        xyz = matrix[:3, 3].tolist()
        q = Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist()
        (transform.transform.translation.x, transform.transform.translation.y,
         transform.transform.translation.z) = xyz
        (transform.transform.rotation.x, transform.transform.rotation.y,
         transform.transform.rotation.z, transform.transform.rotation.w) = q
        (self.static_tf if static else self.tf).sendTransform(transform)

    def publish_offsets(self, header, poses):
        if self.object_id in poses:
            self.send_matrix(header, f'aruco_marker_{self.object_id}',
                             self.object_frame, self.marker_object)
        if self.reference_id not in poses:
            return
        # Preview is a separate child, never base_link (which would create a cycle).
        self.send_matrix(header, f'aruco_marker_{self.reference_id}',
                         self.base_prediction_frame, np.linalg.inv(self.base_reference))
        if not self.offset_valid:
            self.get_logger().warning('Base offset not calibrated: preview only, no base pose/TF',
                                      throttle_duration_sec=10.0)
            return
        if self.base_frame == header.frame_id:
            self.get_logger().error('base_frame must differ from camera optical frame')
            return
        if self.publish_camera_tf:
            camera = self.base_reference @ np.linalg.inv(poses[self.reference_id])
            if self.camera_link_frame == header.frame_id:
                self.get_logger().error('camera_link_frame must differ from camera optical frame')
                return
            link_optical = camera_link_to_optical()
            if self.static_optical_frame != header.frame_id:
                self.send_matrix(header, self.camera_link_frame, header.frame_id, link_optical, True)
                self.static_optical_frame = header.frame_id
            self.send_matrix(header, self.base_frame, self.camera_link_frame,
                             camera @ np.linalg.inv(link_optical))
        # Camera localization needs ID0 only; object pose still requires both in this image.
        if self.object_id not in poses:
            return
        # Only compose observations from the SAME image; never reuse stale detections.
        matrix = base_object(self.base_reference, poses[self.reference_id],
                             poses[self.object_id], self.marker_object)
        pose = PoseStamped()
        pose.header.stamp = header.stamp
        pose.header.frame_id = self.base_frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = matrix[:3, 3].tolist()
        q = Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist()
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = q
        self.object_pose.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
