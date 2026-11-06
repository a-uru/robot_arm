import threading
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
from moveit_msgs.msg import RobotTrajectory
from so101_pick_place.node import PickPlace


def fake_return(monkeypatch, carrying=False):
    monkeypatch.setattr('so101_pick_place.node.rclpy.ok', lambda: True)
    events = []
    p = dict(wait_xyz_m=[0., -.2, .2], gripper_open_rad=.8,
             workspace_min_m=[-.4, -.4, 0.], workspace_max_m=[.4, .4, .4])
    n = SimpleNamespace(p=p, carrying=carrying, motion_fault=True,
                        return_requested=threading.Event(), stop_requested=threading.Event(),
                        return_in_progress=False, cancel_unconfirmed=False,
                        execute=object(), lock=threading.Lock(), busy=True,
                        joint_snapshot=lambda: {'gripper': 0.})
    n.set_status=events.append
    n.apply_scene=lambda *args: events.append('scene')
    n.payload=lambda **kwargs: object()
    def plan(stage, positions):
        from trajectory_msgs.msg import JointTrajectoryPoint
        events.append('plan:'+stage.name)
        result = RobotTrajectory()
        result.joint_trajectory.joint_names = ['gripper']
        result.joint_trajectory.points = [JointTrajectoryPoint(positions=[.8])]
        return result
    n.plan=plan
    n.execute_arm=lambda *args, **kw: events.append('move_wait')
    n.verify=lambda stage: events.append('verify:'+stage.name)
    n.action=lambda *args: events.append('open')
    return n, events


def test_space_cancels_busy_cycle_before_return():
    n=SimpleNamespace(lock=threading.Lock(), return_in_progress=False, busy=True,
                      return_requested=threading.Event(), cancel=Mock())
    assert PickPlace.request_return_to_wait(n)[0]
    assert n.return_requested.is_set()
    n.cancel.assert_called_once()


def test_return_reaches_wait_before_open_and_clears_fault(monkeypatch):
    n, events=fake_return(monkeypatch, carrying=True)
    n.stop_requested.set()
    PickPlace.recover_return(n)
    assert events.index('move_wait') < events.index('verify:wait') < events.index('open')
    assert not n.carrying and not n.motion_fault
    assert not n.return_in_progress
    assert events[-1].startswith('IDLE:')


def test_failed_return_never_opens(monkeypatch):
    n, events=fake_return(monkeypatch, carrying=True)
    def fail(stage):
        raise RuntimeError('wait unreachable')
    n.verify=fail
    PickPlace.recover_return(n)
    assert 'open' not in events
    assert n.carrying and n.motion_fault


def test_unconfirmed_cancel_blocks_return_motion(monkeypatch):
    n, events=fake_return(monkeypatch)
    n.cancel_unconfirmed=True
    PickPlace.recover_return(n)
    assert 'move_wait' not in events and 'open' not in events
    assert n.motion_fault


def test_space_during_return_cancels_instead_of_starting_another():
    n=SimpleNamespace(lock=threading.Lock(), return_in_progress=True, cancel=Mock())
    assert not PickPlace.request_return_to_wait(n)[0]
    n.cancel.assert_called_once()


def test_space_is_immediate_and_terminal_settings_are_restored(monkeypatch):
    import builtins
    import os
    import termios
    master, slave = os.openpty()
    old = termios.tcgetattr(slave)
    real_open = builtins.open
    def terminal_open(path, *args, **kwargs):
        return os.fdopen(os.dup(slave), 'r') if path == '/dev/tty' else real_open(path, *args, **kwargs)
    monkeypatch.setattr(builtins, 'open', terminal_open)
    monkeypatch.setattr('so101_pick_place.node.rclpy.ok', lambda: True)
    space = threading.Event()
    n = SimpleNamespace(keyboard_stop=threading.Event(), stop_requested=threading.Event(), get_logger=lambda: Mock())
    def request_return():
        n.stop_requested.set()
        space.set()
        return True, 'return requested'
    def trigger():
        n.keyboard_stop.set()
        return True, 'accepted'
    n.request_return_to_wait = request_return
    n.trigger = trigger
    worker = threading.Thread(target=PickPlace.keyboard_loop, args=(n,))
    try:
        worker.start()
        import time
        end = time.monotonic()+1
        while termios.tcgetattr(slave) == old and time.monotonic()<end:
            time.sleep(.01)
        os.write(master, b' ')
        assert space.wait(1), 'Space must work without newline'
        os.write(master, b'\n')
        worker.join(timeout=1)
        assert not worker.is_alive()
        assert termios.tcgetattr(slave) == old
    finally:
        n.keyboard_stop.set()
        worker.join(timeout=1)
        os.close(master)
        os.close(slave)
