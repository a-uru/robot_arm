from types import SimpleNamespace
import threading
import numpy as np
from so101_pick_place.node import PickPlace
from so101_pick_place.core import stages


def params():
    return {'wait_xyz_m': [0, -.2, .2], 'transport_clearance_z_m': .15, 'gripper_open_rad': .6, 'gripper_closed_rad': 0., 'approach_height_m': .04,
            'lift_height_m': .06, 'place_approach_height_m': .06, 'retreat_height_m': .06,
            'target_timeout_s': .35, 'max_target_shift_before_grasp_m': .015,
            'grasp_offset_object_m': [0, 0, 0], 'start_joint_tolerance_rad': .06,
            'transit_joint_tolerance_rad': .10, 'grasp_joint_tolerance_rad': .05}


def fake_node():
    events=[]
    node=SimpleNamespace(p=params(), samples=[], lock=threading.Lock(),busy=True,carrying=False,
                         motion_fault=False,execution_started=False,stop_requested=threading.Event(),execute=object(),arm_execute=object())
    node.joint_snapshot=lambda: {'gripper':0.}
    node.apply_scene=lambda *a: events.append('scene')
    node.payload=lambda *a,**kw: object()
    node.set_status=events.append
    node.verify=lambda stage: events.append('verify:'+stage.name)
    node.get_logger=lambda: SimpleNamespace(warning=events.append)
    node.action=lambda *a: events.append('action')
    node.execute_arm = lambda trajectory, precise=True: PickPlace.execute_arm(node, trajectory, precise)
    return node,events


def trajectory():
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    result = RobotTrajectory()
    result.joint_trajectory.joint_names = ['gripper']
    result.joint_trajectory.points = [JointTrajectoryPoint(positions=[0.])]
    return result


def test_failed_preflight_never_executes_or_closes_gripper():
    node,events=fake_node()
    def plan(stage, positions):
        if stage.name=='place':raise RuntimeError('unreachable drop')
        return trajectory()
    node.plan=plan
    PickPlace.cycle(node,np.array([.2,0,.1]),stages([.2,0,.1],[.15,-.1,.08],node.p))
    assert 'action' not in events
    assert not node.carrying
    assert not node.busy
    assert any('unreachable drop' in e for e in events)


def test_close_failure_latches_payload_and_never_releases():
    node,events=fake_node()
    node.plan=lambda *a:trajectory()
    def execute(*args):
        if node.carrying:raise RuntimeError('gripper timeout')
        events.append('executed')
    node.action=execute
    PickPlace.cycle(node,np.array([.2,0,.1]),stages([.2,0,.1],[.15,-.1,.08],node.p))
    assert node.carrying and node.motion_fault
    assert 'EXECUTE release' not in events
    assert 'EXECUTE lift' not in events


def test_replans_from_measured_state_instead_of_preflight_endpoint():
    node, events = fake_node()
    measured = {'gripper': .012}
    calls = []
    node.joint_snapshot = lambda: measured.copy()

    def plan(stage, positions):
        calls.append((stage.name, positions['gripper']))
        # Preflight's hypothetical endpoint must not leak into execution planning.
        positions['gripper'] = .8
        result = trajectory()
        result.joint_trajectory.points[0].positions = [.012]
        return result

    node.plan = plan
    sequence = stages([.2, 0, .1], [.15, -.1, .08], node.p)
    PickPlace.cycle(node, np.array([.2, 0, .1]), sequence)
    assert len(calls) == 2 * len(sequence)
    assert all(position == .012 for _, position in calls[len(sequence):])
    assert any(e.startswith('DONE:') for e in events)


def test_runtime_replan_failure_stops_before_close():
    node, events = fake_node()
    calls = []

    def plan(stage, positions):
        calls.append(stage.name)
        if len(calls) > 8 and stage.name == 'close':
            raise RuntimeError('runtime replan failed')
        return trajectory()

    node.plan = plan
    PickPlace.cycle(node, np.array([.2, 0, .1]), stages([.2, 0, .1], [.15, -.1, .08], node.p))
    assert 'EXECUTE grasp' in events
    assert 'EXECUTE close' not in events
    assert 'EXECUTE lift' not in events
    assert node.motion_fault
    assert not node.carrying


def test_arm_execution_keeps_grasp_strict_and_transit_relaxed():
    from control_msgs.action import FollowJointTrajectory
    from moveit_msgs.action import ExecuteTrajectory
    node, events = fake_node()
    node.plan = lambda *args: trajectory()
    goals = []
    node.action = lambda client, goal: goals.append((client, goal))
    PickPlace.cycle(node, np.array([.2, 0, .1]), stages([.2, 0, .1], [.15, -.1, .08], node.p))
    assert len(goals) == 8
    for index in (1, 4, 5, 7):
        client, goal = goals[index]
        assert client is node.arm_execute
        assert isinstance(goal, FollowJointTrajectory.Goal)
        assert goal.goal_tolerance[0].position == .10
        assert goal.path_tolerance[0].position == .2
    client, goal = goals[2]
    assert client is node.arm_execute
    assert isinstance(goal, FollowJointTrajectory.Goal)
    assert goal.goal_tolerance[0].position == .05
    for index in (0, 3, 6):
        client, goal = goals[index]
        assert client is node.execute
        assert isinstance(goal, ExecuteTrajectory.Goal)


def test_controller_error_is_not_accepted_as_transit_success():
    import pytest
    from unittest.mock import Mock
    from control_msgs.action import FollowJointTrajectory
    from action_msgs.msg import GoalStatus
    for code in (FollowJointTrajectory.Result.SUCCESSFUL,
                 FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED):
        node = SimpleNamespace(p={'action_timeout_s': 10.}, active_goal=None,
                               stop_requested=threading.Event())
        result = SimpleNamespace(error_code=code)
        from rclpy.task import Future
        response = SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result=result)
        terminal = Future()
        terminal.set_result(response)
        handle = Mock(accepted=True)
        handle.get_result_async.return_value = terminal
        future = Mock()
        client = Mock()
        client.send_goal_async.return_value = future
        node.wait_future = lambda item, timeout: handle if item is future else item.result()
        if code == 0:
            assert PickPlace.action(node, client, FollowJointTrajectory.Goal()) is result
        else:
            with pytest.raises(RuntimeError, match='Trajectory/planning failed'):
                PickPlace.action(node, client, FollowJointTrajectory.Goal())
            assert node.stop_requested.is_set()
            handle.cancel_goal_async.assert_called_once()


def test_release_removes_world_copy_after_detach():
    from unittest.mock import Mock
    from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
    requests = []
    node = SimpleNamespace(p={'base_frame': 'base_link'}, scene=Mock())
    node.scene.call_async.side_effect = lambda request: requests.append(request) or SimpleNamespace(success=True)
    node.wait_future = lambda result, timeout: result
    payload = AttachedCollisionObject()
    payload.object.id = 'so101_pick_payload'
    payload.object.operation = CollisionObject.REMOVE
    PickPlace.apply_scene(node, payload)
    assert len(requests) == 2
    assert requests[0].scene.robot_state.attached_collision_objects[0].object.operation == CollisionObject.REMOVE
    assert not requests[0].scene.world.collision_objects
    removed = requests[1].scene.world.collision_objects[0]
    assert removed.id == 'so101_pick_payload'
    assert removed.operation == CollisionObject.REMOVE
    assert not requests[1].scene.robot_state.attached_collision_objects


def test_space_interrupt_runs_return_after_cycle_unwinds():
    node, events = fake_node()
    node.return_requested = threading.Event()
    node.plan = lambda *args: trajectory()
    def interrupted(*args):
        node.return_requested.set()
        node.stop_requested.set()
        raise RuntimeError('Cancelled')
    node.action = interrupted
    node.recover_return = lambda: events.append('recover_after_cancel')
    PickPlace.cycle(node, np.array([.2, 0, .1]), stages([.2, 0, .1], [.15, -.1, .08], node.p))
    assert events[-1] == 'recover_after_cancel'
    assert any(e.startswith('ABORTED: Cancelled') for e in events)
    assert not node.busy and not node.return_requested.is_set()
