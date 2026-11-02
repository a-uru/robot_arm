import numpy as np
import pytest
from so101_servo.kinematics import Chain, tracking_velocity

URDF = """<robot name="test">
<joint name="a" type="revolute"><parent link="base"/><child link="one"/>
<origin xyz="0.1 0 0" rpy="0.1 0.2 0.3"/><axis xyz="0 0 1"/></joint>
<joint name="b" type="revolute"><parent link="one"/><child link="two"/>
<origin xyz="0.2 0 0" rpy="0 0.2 0"/><axis xyz="0 1 0"/></joint>
<joint name="c" type="revolute"><parent link="two"/><child link="three"/>
<origin xyz="0.1 0 0.1"/><axis xyz="1 0 0"/></joint>
<joint name="tool" type="fixed"><parent link="three"/><child link="tip"/>
<origin xyz="0.1 0.1 0.1"/></joint></robot>"""


def test_jacobian_matches_finite_difference_including_fixed_tool_and_rpy():
    chain = Chain(URDF, "base", "tip")
    q = {"a": .3, "b": -.4, "c": .2}
    point, jac = chain.evaluate(q)
    for i, name in enumerate(chain.names):
        other = dict(q)
        other[name] += 1e-6
        moved, _ = chain.evaluate(other)
        np.testing.assert_allclose((moved-point)/1e-6, jac[:, i], atol=1e-6)


def test_position_control_reduces_error_with_redundant_joints():
    jac = np.array([[1., 0, 0, .1, 0], [0, 1., 0, .2, 0], [0, 0, 1., .3, 0]])
    error = np.array([.01, -.02, .01])
    dq = tracking_velocity(error, jac, 2., .03, .02, .3)
    assert np.dot(error, jac @ dq) > 0
    assert np.linalg.norm(error-.02*jac@dq) < np.linalg.norm(error)
    assert abs(dq[-1]) < 1e-12  # A position-irrelevant wrist axis is free.


def test_joint_speed_and_cartesian_speed_are_bounded():
    jac = np.eye(3)
    dq = tracking_velocity([10., 10., 10.], jac, 2., .03, .02, .01)
    assert np.max(np.abs(dq)) <= .01+1e-12
    assert np.linalg.norm(jac@dq) <= .03+1e-12


def test_nonfinite_target_rejected():
    with pytest.raises(ValueError):
        tracking_velocity([float("nan"), 0, 0], np.eye(3), 2., .03, .02, .3)


def test_disconnected_tip_rejected():
    with pytest.raises(ValueError):
        Chain(URDF, "base", "missing")


def test_offset_jacobian_matches_finite_difference():
    chain = Chain(URDF, "base", "tip")
    q = {"a": .4, "b": -.2, "c": .3}
    offset = [.03, -.02, .08]
    point, jac = chain.evaluate(q, offset)
    for i, name in enumerate(chain.names):
        moved = dict(q)
        moved[name] += 1e-6
        next_point, _ = chain.evaluate(moved, offset)
        np.testing.assert_allclose((next_point-point)/1e-6, jac[:, i], atol=1e-6)
    assert np.linalg.norm(point-chain.evaluate(q)[0]) > .05


def test_local_offset_rotates_with_tool():
    from so101_servo.kinematics import offset_point
    result = offset_point([1, 2, 3], [0, 0, np.sqrt(.5), np.sqrt(.5)], [.1, 0, 0])
    np.testing.assert_allclose(result, [1, 2.1, 3], atol=1e-12)


def test_nonfinite_offset_rejected():
    chain = Chain(URDF, "base", "tip")
    with pytest.raises(ValueError):
        chain.evaluate({"a": 0, "b": 0, "c": 0}, [0, float("nan"), 0])


def test_rank_deficient_jacobian_preserves_reachable_motion():
    jac = np.array([[1., 0, 0, 0, 0], [0, 1., 0, 0, 0], [0, 0, 0, 0, 0]])
    dq = tracking_velocity([.01, .01, .02], jac, 4., .08, .02, .8)
    assert np.all(np.isfinite(dq))
    assert np.linalg.norm(jac@dq) > 0
    assert np.dot(np.array([.01, .01, .02]), jac@dq) > 0
    assert np.max(np.abs(dq)) <= .8


def test_rank_deficient_jacobian_does_not_invent_escape_motion():
    jac = np.array([[1., 0, 0, 0, 0], [0, 1., 0, 0, 0], [0, 0, 0, 0, 0]])
    dq = tracking_velocity([0, 0, .02], jac, 4., .08, .02, .8)
    np.testing.assert_allclose(dq, np.zeros(5), atol=1e-12)
