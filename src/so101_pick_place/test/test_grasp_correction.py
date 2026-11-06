from types import SimpleNamespace
import threading
import numpy as np
import pytest
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from so101_pick_place.core import Stage
from so101_pick_place.node import PickPlace


def make_node(errors):
    sequence = iter([errors[0]] + errors)
    commands = []
    p = dict(grasp_correction_attempts=3, grasp_correction_settle_s=.001,
             reached_tolerance_m=.005, tcp_offset_m=[0, 0, 0],
             target_timeout_s=.35, grasp_offset_object_m=[0, 0, 0],
             max_target_shift_before_grasp_m=.015, grasp_correction_gain=.7,
             grasp_correction_step_m=.008, grasp_correction_max_offset_m=.025,
             grasp_correction_max_joint_step_rad=.15,
             workspace_min_m=[-.4, -.4, -.4], workspace_max_m=[.4, .4, .4])
    n = SimpleNamespace(p=p, stop_requested=threading.Event(), lock=threading.Lock(), samples=[],
                        joint_snapshot=lambda: {'elbow_flex': 0.}, set_status=lambda text: None)
    n.chain = SimpleNamespace(evaluate=lambda *args: (np.array(next(sequence)), None))

    def plan(stage, positions):
        commands.append(np.array(stage.xyz))
        t = RobotTrajectory()
        t.joint_trajectory.joint_names = ['elbow_flex']
        t.joint_trajectory.points = [JointTrajectoryPoint(positions=[.01])]
        return t

    n.plan = plan
    n.execute_arm = lambda *args, **kwargs: None
    return n, commands


def test_already_accurate_needs_no_replanning():
    n, commands = make_node([[.001, 0, 0]])
    PickPlace.correct_grasp(n, Stage('grasp', np.zeros(3)))
    assert not commands


def test_compensation_accumulates_but_verifies_original_target():
    n, commands = make_node([[.016, 0, 0], [.007, 0, 0], [.003, 0, 0]])
    PickPlace.correct_grasp(n, Stage('grasp', np.zeros(3)))
    assert np.allclose(commands[0], [-.008, 0, 0])
    assert np.allclose(commands[1], [-.0129, 0, 0])


def test_worsening_error_aborts():
    n, commands = make_node([[.016, 0, 0], [.020, 0, 0]])
    with pytest.raises(RuntimeError, match='worsened'):
        PickPlace.correct_grasp(n, Stage('grasp', np.zeros(3)))
    assert len(commands) == 1


def test_workspace_violation_never_executes():
    n, commands = make_node([[.016, 0, 0]])
    n.p['workspace_min_m'] = [0., -.4, -.4]
    with pytest.raises(ValueError, match='outside workspace'):
        PickPlace.correct_grasp(n, Stage('grasp', np.zeros(3)))
    assert not commands


def test_attempt_limit_stops_unresponsive_arm():
    n, commands = make_node([[.016, 0, 0]]*4)
    with pytest.raises(RuntimeError, match='exhausted'):
        PickPlace.correct_grasp(n, Stage('grasp', np.zeros(3)))
    assert len(commands) == 3


def test_total_offset_limit_stops_before_plan():
    n, commands = make_node([[.016, 0, 0]])
    n.p['grasp_correction_max_offset_m'] = .005
    with pytest.raises(RuntimeError, match='total offset'):
        PickPlace.correct_grasp(n, Stage('grasp', np.zeros(3)))
    assert not commands


def test_cancel_during_settle_never_plans():
    n, commands = make_node([[.016, 0, 0]])
    n.stop_requested.set()
    with pytest.raises(RuntimeError, match='Cancelled'):
        PickPlace.correct_grasp(n, Stage('grasp', np.zeros(3)))
    assert not commands
