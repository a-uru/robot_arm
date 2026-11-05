import numpy as np
import pytest
from so101_aruco.transforms import from_xyz_rpy, base_object


def test_inverse_includes_rotation():
    transform = from_xyz_rpy([.1, .2, .03], [0, 0, 90])
    np.testing.assert_allclose(np.linalg.inv(transform)[:3, 3], [-.2, .1, -.03], atol=1e-12)


def test_full_chain_recovers_known_base_object():
    base_camera = from_xyz_rpy([.4, -.1, .7], [150, 10, -20])
    base_reference = from_xyz_rpy([.03, .04, .05], [90, 0, 45])
    expected_object = from_xyz_rpy([.2, -.15, .08], [0, 10, 30])
    marker_object = from_xyz_rpy([.01, .02, -.015], [0, 0, 90])
    camera_reference = np.linalg.inv(base_camera) @ base_reference
    camera_marker = np.linalg.inv(base_camera) @ expected_object @ np.linalg.inv(marker_object)
    np.testing.assert_allclose(base_object(base_reference, camera_reference, camera_marker, marker_object),
                               expected_object, atol=1e-12)


def test_offset_validation():
    with pytest.raises(ValueError):
        from_xyz_rpy([float('nan'), 0, 0], [0, 0, 0])


def test_camera_motion_cancels_for_stationary_object():
    reference = from_xyz_rpy([.01, .03, .05], [90, 0, 20])
    expected = from_xyz_rpy([.2, -.1, .04], [0, 0, 35])
    offset = from_xyz_rpy([0, 0, -.01], [0, 0, 0])
    for xyz, rpy in [([.1, .2, .4], [170, 10, 20]), ([-.2, .3, .6], [140, -15, 40])]:
        base_camera = from_xyz_rpy(xyz, rpy)
        observed_ref = np.linalg.inv(base_camera) @ reference
        observed_obj = np.linalg.inv(base_camera) @ expected @ np.linalg.inv(offset)
        result = base_object(reference, observed_ref, observed_obj, offset)
        np.testing.assert_allclose(result, expected, atol=1e-12)


def test_camera_link_preserves_optical_pose_and_axis_conventions():
    from so101_aruco.transforms import camera_link_to_optical
    link_optical = camera_link_to_optical()
    np.testing.assert_allclose(link_optical[:3, :3] @ [0, 0, 1], [1, 0, 0])
    np.testing.assert_allclose(link_optical[:3, :3] @ [1, 0, 0], [0, -1, 0])
    np.testing.assert_allclose(link_optical[:3, :3] @ [0, 1, 0], [0, 0, -1])
    base_optical = from_xyz_rpy([.2, .1, .4], [170, -10, 35])
    base_link = base_optical @ np.linalg.inv(link_optical)
    np.testing.assert_allclose(base_link @ link_optical, base_optical, atol=1e-12)
