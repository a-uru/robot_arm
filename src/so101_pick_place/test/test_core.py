import numpy as np
import pytest
from so101_pick_place.core import stages, stable_snapshot, validate_stages, grasp_point


def config():
    return {'wait_xyz_m': [0, -.2, .2], 'transport_clearance_z_m': .15, 'gripper_open_rad': .6, 'gripper_closed_rad': 0., 'approach_height_m': .04,
            'lift_height_m': .06, 'place_approach_height_m': .06, 'retreat_height_m': .06,
            'target_timeout_s': .35, 'stable_window_s': .5, 'stable_min_samples': 5,
            'grasp_offset_object_m': [0, 0, 0], 'stable_position_std_m': .004,
            'stable_position_range_m': .015}


def test_sequence_and_attached_state():
    result = stages([.15, .1, .05], [.15, -.1, .08], config())
    assert [s.name for s in result] == ['open','approach','grasp','close','clearance','place','release','wait']
    assert [s.name for s in result if s.carrying] == ['clearance','place','release']
    np.testing.assert_allclose(result[1].xyz, [.15, .1, .09])
    np.testing.assert_allclose(result[4].xyz, [.15, .1, .15])
    np.testing.assert_allclose(result[5].xyz, [.15, -.1, .08])
    assert result[6].gripper == .6
    np.testing.assert_allclose(result[7].xyz, [0., -.2, .2])
    assert not result[7].carrying


def test_rotated_grasp_offset():
    p = grasp_point([.1, .2, .3], [0, 0, np.sqrt(.5), np.sqrt(.5)], [.01, 0, 0])
    np.testing.assert_allclose(p, [.1, .21, .3], atol=1e-12)


def test_unreachable_destination_rejected_before_any_plan():
    with pytest.raises(ValueError, match='outside workspace'):
        validate_stages(stages([.15,.1,.05], [.8,0,.1], config()), [0,-.3,0], [.4,.3,.4])


def samples():
    return [(9.55+i*.05, 100.+i*.05, [.15,.1,.05], [0,0,0,1]) for i in range(10)]


def test_stable_snapshot_rejects_stale_and_moving_targets():
    p=config()
    np.testing.assert_allclose(stable_snapshot(samples(),10.,p), [.15,.1,.05])
    with pytest.raises(ValueError, match='fresh'):
        stable_snapshot(samples(),11.,p)
    moving=samples();moving[-1]=(10.,100.5,[.2,.1,.05],[0,0,0,1])
    with pytest.raises(ValueError, match='unstable'):
        stable_snapshot(moving,10.,p)


def test_repeated_timestamp_and_short_window_rejected():
    s=samples()
    with pytest.raises(ValueError):stable_snapshot(s[-2:],10.,config())
    s=[(t,100.,p,q) for t,stamp,p,q in s]
    with pytest.raises(ValueError, match='timestamps'):
        stable_snapshot(s,10.,config())
