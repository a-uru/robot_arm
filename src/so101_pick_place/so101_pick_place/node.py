"""Preplanned, Enter-triggered pick/place; no motion on startup."""
import os
import select
import threading
import time
from collections import deque
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from builtin_interfaces.msg import Duration
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject, Constraints,
                             JointConstraint, MoveItErrorCodes, RobotState)
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
from so101_servo.kinematics import Chain
from .ik import tcp_candidates
from .core import grasp_point, stable_snapshot, stages, validate_stages, vector


class PickPlace(Node):
    def __init__(self):
        super().__init__('so101_pick_place')
        path = os.path.join(get_package_share_directory('so101_pick_place'), 'config', 'pick_place.yaml')
        with open(path) as stream:
            defaults = yaml.safe_load(stream)['so101_pick_place']['ros__parameters']
        for key, val in defaults.items():
            self.declare_parameter(key, val)
        self.declare_parameter('use_fake_hardware', True)
        self.declare_parameter('robot_description', '')
        self.p = {k: self.get_parameter(k).value for k in defaults}
        self.fake = self.get_parameter('use_fake_hardware').value
        xml = ET.fromstring(self.get_parameter('robot_description').value)
        self.limits = {}
        for j in xml.findall('joint'):
            lim = j.find('limit')
            if j.get('type') in ('revolute', 'prismatic') and lim is not None:
                self.limits[j.get('name')] = (float(lim.get('lower')), float(lim.get('upper')))
        self.arm_joints = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
        self.joints = self.arm_joints + ['gripper']
        for name in self.joints:
            if name not in self.limits:
                raise ValueError(f'Missing model joint {name}')
        if self.p['tip_link'] not in {e.get('name') for e in xml.findall('link')}:
            raise ValueError('tip_link is absent from robot model')
        self.chain = Chain(self.get_parameter('robot_description').value, self.p['base_frame'], self.p['tip_link'])
        if set(self.chain.names) != set(self.arm_joints):
            raise ValueError('tip_link must be on the arm chain after wrist_roll')
        self._validate()
        self.tcp_broadcaster = StaticTransformBroadcaster(self)
        tcp_tf = TransformStamped()
        tcp_tf.header.stamp = self.get_clock().now().to_msg()
        tcp_tf.header.frame_id = self.p['tip_link']
        tcp_tf.child_frame_id = 'tcp_link'
        tcp_tf.transform.translation.x = float(self.p['tcp_offset_m'][0])
        tcp_tf.transform.translation.y = float(self.p['tcp_offset_m'][1])
        tcp_tf.transform.translation.z = float(self.p['tcp_offset_m'][2])
        tcp_tf.transform.rotation.w = 1.0
        self.tcp_broadcaster.sendTransform(tcp_tf)
        self.lock = threading.Lock()
        self.samples = deque(maxlen=300)
        self.current_joints = {}
        self.joint_receipt = 0.
        self.busy = False
        self.active_goal = None
        self.stop_requested = threading.Event()
        self.carrying = False
        self.motion_fault = False
        self.execution_started = False
        self.status = self.create_publisher(String, '/pick_place/status', 10)
        self.create_subscription(PoseStamped, '/aruco/object/pose', self.on_target, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_service(Trigger, '/pick_place/trigger', self.on_trigger)
        self.create_service(Trigger, '/pick_place/cancel', self.on_cancel)
        self.move_group = ActionClient(self, MoveGroup, '/move_action')
        self.execute = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.arm_execute = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.validity = self.create_client(GetStateValidity, '/check_state_validity')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.set_status('IDLE: Enter to snapshot a fresh stable target and plan all stages')
        if not self.p['configuration_ready']:
            self.get_logger().warning('Set place_xyz_m, gripper angles and TCP in pick_place.yaml, then configuration_ready=true. Enter is disabled meanwhile.')
        self.worker = None
        if self.p['keyboard_enabled']:
            threading.Thread(target=self.keyboard_loop, daemon=True).start()

    def _validate(self):
        for k in ('tcp_offset_m', 'grasp_offset_object_m', 'place_xyz_m', 'workspace_min_m', 'workspace_max_m', 'table_size_m'):
            vector(self.p[k], k)
        if len(self.p['table_center_xy_m']) != 2 or not np.isfinite(self.p['table_center_xy_m']).all():
            raise ValueError('table_center_xy_m must be finite XY')
        for k in ('approach_height_m', 'lift_height_m', 'place_approach_height_m', 'retreat_height_m',
                  'position_tolerance_m', 'reached_tolerance_m', 'planning_time_s', 'action_timeout_s',
                  'target_timeout_s', 'stable_window_s', 'stable_position_std_m', 'stable_position_range_m',
                  'joint_state_timeout_s', 'max_target_shift_before_grasp_m', 'start_joint_tolerance_rad',
                  'gripper_reached_tolerance_rad', 'payload_radius_m',
                  'transit_reached_tolerance_m', 'transit_joint_tolerance_rad', 'grasp_joint_tolerance_rad'):
            if not np.isfinite(self.p[k]) or self.p[k] <= 0:
                raise ValueError(f'{k} must be finite and positive')
        if self.p['stable_min_samples'] < 2 or np.any(np.array(self.p['table_size_m']) <= 0):
            raise ValueError('Invalid stable_min_samples/table_size_m')
        if not np.isfinite(self.p['table_top_z_m']):
            raise ValueError('table_top_z_m must be finite')
        for k in ('velocity_scaling', 'acceleration_scaling'):
            if not 0 < self.p[k] <= 1:
                raise ValueError(f'{k} must be in (0, 1]')
        lo, hi = self.limits['gripper']
        for k in ('gripper_open_rad', 'gripper_closed_rad'):
            if not np.isfinite(self.p[k]) or not lo <= self.p[k] <= hi:
                raise ValueError(f'{k} must be within URDF bounds {lo}..{hi} rad')
        if abs(self.p['gripper_open_rad'] - self.p['gripper_closed_rad']) < .01:
            raise ValueError('Open and closed gripper angles must differ')
        validate_stages(stages(self.p['place_xyz_m'], self.p['place_xyz_m'], self.p),
                        self.p['workspace_min_m'], self.p['workspace_max_m'])

    def set_status(self, text):
        self.status.publish(String(data=text))
        self.get_logger().info(text)

    def on_target(self, msg):
        age = self.get_clock().now().nanoseconds / 1e9 - (msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)
        if msg.header.frame_id != self.p['base_frame'] or age < -.1 or age > self.p['target_timeout_s']:
            return
        p, q = msg.pose.position, msg.pose.orientation
        xyz, quat = [p.x, p.y, p.z], [q.x, q.y, q.z, q.w]
        if not np.isfinite(xyz + quat).all() or np.linalg.norm(quat) < 1e-6:
            return
        sample = (time.monotonic(), msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9, xyz, quat)
        with self.lock:
            if self.samples and sample[1] <= self.samples[-1][1]:
                return
            self.samples.append(sample)

    def on_joints(self, msg):
        age = self.get_clock().now().nanoseconds / 1e9 - (msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)
        if not -.1 <= age <= self.p['joint_state_timeout_s']:
            return
        values = dict(zip(msg.name, msg.position))
        if all(j in values and np.isfinite(values[j]) for j in self.joints):
            with self.lock:
                self.current_joints = {j: float(values[j]) for j in self.joints}
                self.joint_receipt = time.monotonic()

    def joint_snapshot(self):
        with self.lock:
            if time.monotonic() - self.joint_receipt > self.p['joint_state_timeout_s'] or not self.current_joints:
                raise RuntimeError('No fresh complete joint feedback')
            return self.current_joints.copy()

    def keyboard_loop(self):
        try:
            with open('/dev/tty', 'r') as terminal:
                while rclpy.ok() and not self.stop_requested.is_set():
                    if select.select([terminal], [], [], .2)[0]:
                        line = terminal.readline()
                        if line == '':
                            return
                        if not line.strip():
                            accepted, message = self.trigger()
                            if not accepted:
                                self.get_logger().warning(message)
        except OSError:
            self.get_logger().warning('No controlling terminal: use /pick_place/trigger service')

    def on_trigger(self, request, response):
        response.success, response.message = self.trigger()
        return response

    def trigger(self):
        with self.lock:
            if self.busy:
                return False, 'BUSY: additional Enter ignored'
            if not self.p['configuration_ready']:
                return False, 'configuration_ready=false; edit pick_place.yaml and restart'
            if not self.fake and not self.p['tcp_offset_confirmed']:
                return False, 'Measure TCP offset and set tcp_offset_confirmed=true for real hardware'
            if self.motion_fault:
                return False, 'Execution fault/cancel latched; inspect robot state and restart'
            if self.carrying:
                return False, 'Payload may still be held after failure; resolve manually and restart'
            try:
                target = stable_snapshot(list(self.samples), time.monotonic(), self.p)
                latest_stamp = self.samples[-1][1]
                age = self.get_clock().now().nanoseconds / 1e9 - latest_stamp
                if not -.1 <= age <= self.p['target_timeout_s']:
                    raise ValueError('Latest pose timestamp is stale')
                sequence = stages(target, self.p['place_xyz_m'], self.p)
                validate_stages(sequence, self.p['workspace_min_m'], self.p['workspace_max_m'])
            except ValueError as exc:
                return False, str(exc)
            self.busy = True
            self.stop_requested.clear()
        self.worker = threading.Thread(target=self.cycle, args=(target, sequence), daemon=True)
        self.worker.start()
        return True, f'Target frozen: {target.tolist()}; planning all stages before motion'

    def on_cancel(self, request, response):
        self.cancel()
        response.success, response.message = True, 'Cancellation requested; no automatic gripper release'
        return response

    def cancel(self):
        self.stop_requested.set()
        goal = self.active_goal
        if goal is not None:
            goal.cancel_goal_async()

    def wait_future(self, future, timeout):
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        deadline = time.monotonic() + timeout
        while not event.wait(.05):
            if self.stop_requested.is_set() or not rclpy.ok():
                raise RuntimeError('Cancelled')
            if time.monotonic() > deadline:
                raise RuntimeError('ROS request/action timed out')
        if self.stop_requested.is_set():
            raise RuntimeError('Cancelled')
        return future.result()

    def action(self, client, goal):
        if not client.wait_for_server(timeout_sec=5):
            raise RuntimeError('MoveIt action server unavailable')
        # If acceptance arrives after a timeout/cancel, cancel the late goal as well.
        future = client.send_goal_async(goal)
        def late_cancel(f):
            handle = f.result()
            if self.stop_requested.is_set() and handle and handle.accepted:
                handle.cancel_goal_async()
        future.add_done_callback(late_cancel)
        handle = None
        try:
            handle = self.wait_future(future, 10.)
            if not handle.accepted:
                raise RuntimeError('MoveIt rejected action goal')
            self.active_goal = handle
            response = self.wait_future(handle.get_result_async(), self.p['action_timeout_s'])
            direct = isinstance(goal, FollowJointTrajectory.Goal)
            code = response.result.error_code if direct else response.result.error_code.val
            success = FollowJointTrajectory.Result.SUCCESSFUL if direct else MoveItErrorCodes.SUCCESS
            if response.status != GoalStatus.STATUS_SUCCEEDED or code != success:
                raise RuntimeError(f'Trajectory/planning failed: action={response.status}, code={code}')
            return response.result
        except Exception:
            self.stop_requested.set()
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            raise
        finally:
            self.active_goal = None

    def payload(self, remove=False):
        attached = AttachedCollisionObject()
        attached.link_name = self.p['tip_link']
        attached.touch_links = ['gripper_link', 'jaw_link']
        obj = attached.object
        obj.id = 'so101_pick_payload'
        obj.header.frame_id = self.p['tip_link']
        obj.operation = CollisionObject.REMOVE if remove else CollisionObject.ADD
        if not remove:
            shape = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[float(self.p['payload_radius_m'] + np.linalg.norm(self.p['grasp_offset_object_m']))])
            pose = Pose()
            pose.orientation.w = 1.
            pose.position.x, pose.position.y, pose.position.z = self.p['tcp_offset_m']
            obj.primitives = [shape]
            obj.primitive_poses = [pose]
        return attached

    def apply_scene(self, payload=None):
        if payload is None and not self.p['table_enabled']:
            return
        if not self.scene.wait_for_service(timeout_sec=5):
            raise RuntimeError('Planning scene service unavailable')
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        if payload is not None:
            request.scene.robot_state.attached_collision_objects = [payload]
        else:
            obj = CollisionObject()
            obj.id = 'so101_pick_table'
            obj.header.frame_id = self.p['base_frame']
            obj.operation = CollisionObject.ADD if self.p['table_enabled'] else CollisionObject.REMOVE
            if self.p['table_enabled']:
                size = self.p['table_size_m']
                shape = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=size)
                pose = Pose()
                pose.orientation.w = 1.
                pose.position.x, pose.position.y = self.p['table_center_xy_m']
                pose.position.z = self.p['table_top_z_m'] - size[2] / 2
                obj.primitives, obj.primitive_poses = [shape], [pose]
            request.scene.world.collision_objects = [obj]
        result = self.wait_future(self.scene.call_async(request), 5.)
        if not result.success:
            raise RuntimeError('Planning scene update failed')
        if payload is not None and payload.object.operation == CollisionObject.REMOVE:
            # Detaching moves the attached geometry into the world. Remove this
            # grasp-only approximation AFTER the detach has been applied.
            cleanup = ApplyPlanningScene.Request()
            cleanup.scene.is_diff = True
            cleanup.scene.robot_state.is_diff = True
            obj = CollisionObject()
            obj.id = payload.object.id
            obj.header.frame_id = self.p['base_frame']
            obj.operation = CollisionObject.REMOVE
            cleanup.scene.world.collision_objects = [obj]
            if not self.wait_future(self.scene.call_async(cleanup), 5.).success:
                raise RuntimeError('Released payload world cleanup failed')

    def solve_tcp(self, stage, positions):
        if not self.validity.wait_for_service(timeout_sec=5):
            raise RuntimeError('MoveIt state validity service unavailable')
        for solution in tcp_candidates(self.chain, positions, self.p['tcp_offset_m'], stage.xyz,
                                       self.limits, self.p['position_tolerance_m'], self.p['planning_time_s']):
            combined = {**positions, **solution}
            request = GetStateValidity.Request()
            request.robot_state.joint_state.name = self.joints
            request.robot_state.joint_state.position = [combined[j] for j in self.joints]
            request.robot_state.is_diff = False
            if stage.carrying:
                request.robot_state.attached_collision_objects = [self.payload()]
            if self.wait_future(self.validity.call_async(request), 5.).valid:
                return solution
        raise RuntimeError(f'{stage.name}: no collision-free position-only TCP IK solution')

    def plan(self, stage, positions):
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = 'arm' if stage.xyz is not None else 'gripper'
        request.pipeline_id = 'ompl'
        request.num_planning_attempts = 5
        request.allowed_planning_time = self.p['planning_time_s']
        request.max_velocity_scaling_factor = self.p['velocity_scaling']
        request.max_acceleration_scaling_factor = self.p['acceleration_scaling']
        request.start_state = RobotState()
        request.start_state.is_diff = False
        request.start_state.joint_state.name = self.joints
        request.start_state.joint_state.position = [positions[j] for j in self.joints]
        if stage.carrying:
            request.start_state.attached_collision_objects = [self.payload()]
        constraints = Constraints(name=stage.name)
        if stage.xyz is not None:
            solution = self.solve_tcp(stage, positions)
            constraints.joint_constraints = [JointConstraint(joint_name=j, position=solution[j],
                tolerance_above=.002, tolerance_below=.002, weight=1.) for j in self.arm_joints]
        else:
            constraints.joint_constraints = [JointConstraint(joint_name='gripper', position=float(stage.gripper),
                tolerance_above=.01, tolerance_below=.01, weight=1.)]
        request.goal_constraints = [constraints]
        goal.planning_options.plan_only = True
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        result = self.action(self.move_group, goal)
        trajectory = result.planned_trajectory
        if trajectory.multi_dof_joint_trajectory.points:
            raise RuntimeError('Unexpected multi-DOF trajectory')
        if trajectory.joint_trajectory.points:
            end = trajectory.joint_trajectory.points[-1]
            positions.update(zip(trajectory.joint_trajectory.joint_names, end.positions))
        return trajectory

    def verify(self, stage):
        # Allow a short settling interval, continuously checking actual feedback.
        deadline = time.monotonic() + 3.
        last = 'No feedback'
        while time.monotonic() < deadline:
            if self.stop_requested.is_set():
                raise RuntimeError('Cancelled')
            try:
                measured = self.joint_snapshot()
                if stage.xyz is None:
                    if abs(measured['gripper'] - stage.gripper) <= self.p['gripper_reached_tolerance_rad']:
                        return
                    last = 'Gripper did not reach requested joint angle'
                else:
                    transform = self.buffer.lookup_transform(self.p['base_frame'], self.p['tip_link'], Time())
                    age = self.get_clock().now().nanoseconds/1e9 - (transform.header.stamp.sec + transform.header.stamp.nanosec/1e9)
                    if not -.1 <= age <= self.p['joint_state_timeout_s']:
                        raise RuntimeError('Stale end-effector TF')
                    t, q = transform.transform.translation, transform.transform.rotation
                    tcp = grasp_point([t.x,t.y,t.z],[q.x,q.y,q.z,q.w],self.p['tcp_offset_m'])
                    tolerance = self.p['reached_tolerance_m'] if stage.name == 'grasp' else self.p['transit_reached_tolerance_m']
                    if np.linalg.norm(tcp - stage.xyz) <= tolerance:
                        return
                    last = f'TCP missed target: {np.linalg.norm(tcp-stage.xyz):.4f} m'
            except Exception as exc:
                last = str(exc)
            time.sleep(.05)
        raise RuntimeError(last)

    def cycle(self, target, sequence):
        self.execution_started = False
        try:
            self.joint_snapshot()
            self.apply_scene()
            positions = self.joint_snapshot()
            self.set_status(f'PLANNING: frozen target {target.tolist()}')
            for stage in sequence:
                if self.stop_requested.is_set():
                    raise RuntimeError('Cancelled')
                self.set_status('PLAN ' + stage.name)
                self.plan(stage, positions)
            self.set_status('All stages preflighted; replanning each stage from measured joints')
            for stage in sequence:
                if self.stop_requested.is_set():
                    raise RuntimeError('Cancelled')
                self.set_status('REPLAN ' + stage.name)
                trajectory = self.plan(stage, self.joint_snapshot())
                if self.stop_requested.is_set():
                    raise RuntimeError('Cancelled')
                if stage.name in ('grasp', 'close'):
                    with self.lock:
                        latest = self.samples[-1] if self.samples else None
                    if latest and time.monotonic() - latest[0] <= self.p['target_timeout_s']:
                        observed = grasp_point(latest[2], latest[3], self.p['grasp_offset_object_m'])
                        shift = float(np.linalg.norm(observed - target))
                        if shift > self.p['max_target_shift_before_grasp_m']:
                            raise RuntimeError(
                                f'{stage.name}: object estimate shifted {shift:.4f} m '
                                f'(limit {self.p["max_target_shift_before_grasp_m"]:.4f} m); '
                                f'Enter target={target.tolist()}, observed={observed.tolist()}. '
                                'Possible object motion or pose-estimation drift; cycle aborted')
                points = trajectory.joint_trajectory.points
                if points:
                    current = self.joint_snapshot()
                    for joint, expected in zip(trajectory.joint_trajectory.joint_names, points[0].positions):
                        if abs(current[joint] - expected) > self.p['start_joint_tolerance_rad']:
                            raise RuntimeError(f'{stage.name}: start joint state changed ({joint})')
                self.set_status('EXECUTE ' + stage.name)
                self.execution_started = True
                # Close may physically hold an object even if feedback/result subsequently fails.
                if stage.name == 'close':
                    self.carrying = True
                if points:
                    if stage.xyz is not None:
                        goal = FollowJointTrajectory.Goal()
                        goal.trajectory = trajectory.joint_trajectory
                        joint_tolerance = self.p['grasp_joint_tolerance_rad'] if stage.name == 'grasp' else self.p['transit_joint_tolerance_rad']
                        goal.goal_tolerance = [JointTolerance(name=j, position=joint_tolerance)
                                               for j in trajectory.joint_trajectory.joint_names]
                        goal.path_tolerance = [JointTolerance(name=j, position=.2)
                                               for j in trajectory.joint_trajectory.joint_names]
                        goal.goal_time_tolerance = Duration(sec=2)
                        self.action(self.arm_execute, goal)
                    else:
                        goal = ExecuteTrajectory.Goal()
                        goal.trajectory = trajectory
                        goal.controller_names = ['arm_controller' if stage.xyz is not None else 'gripper_controller']
                        self.action(self.execute, goal)
                self.verify(stage)
                if stage.name == 'close':
                    self.apply_scene(self.payload())
                if stage.name == 'release':
                    self.apply_scene(self.payload(remove=True))
                    self.carrying = False
            self.set_status('DONE: placed and released; Enter starts a new cycle')
        except Exception as exc:
            if self.execution_started:
                self.motion_fault = True
            self.set_status(f'ABORTED: {exc}. No later stage or automatic gripper release.')
            if self.carrying:
                self.get_logger().warning('Object may be held; manual recovery required before next cycle')
        finally:
            with self.lock:
                self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = PickPlace()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel()
        if node.worker:
            node.worker.join(timeout=2.)
        executor.shutdown(timeout_sec=2.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
