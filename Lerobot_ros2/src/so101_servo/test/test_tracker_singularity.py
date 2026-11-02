"""Regression: a rank-deficient position Jacobian must not stop all motion."""
import time
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
from builtin_interfaces.msg import Time
from so101_servo.point_tracker import PointTracker


def test_tracker_tick_continues_reachable_motion_at_rank_deficient_pose():
    now = time.monotonic()
    jac = np.array([[1., 0, 0, 0, 0], [0, 1., 0, 0, 0], [0, 0, 0, 0, 0]])
    tracker = SimpleNamespace(
        last_tick=now-.02, mode_ready=True, state_received=now, target_received=now,
        target=object(), positions={}, offset=np.zeros(3),
        p={"publish_rate": 50., "state_timeout": .2, "target_timeout": .3,
           "base_frame": "base_link", "position_tolerance": .003,
           "gain": 4., "max_linear_speed": .08, "damping": .02,
           "max_joint_speed": .8, "max_joint_acceleration": 1.},
        chain=SimpleNamespace(names=list("abcde"), evaluate=lambda *args: (np.zeros(3), jac)),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time())),
        control_point_pub=Mock(), error_pub=Mock(),
        target_in_base=lambda: np.array([.01, 0, .02]),
        last_velocity=np.zeros(5), publish=Mock(), set_status=Mock(), stop=Mock(),
    )
    PointTracker.tick(tracker)
    tracker.stop.assert_not_called()
    tracker.set_status.assert_called_with("tracking_near_singularity")
    assert np.linalg.norm(tracker.publish.call_args.args[0]) > 0
    assert np.max(np.abs(tracker.last_velocity)) <= .04
