import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from so101_aruco.vision import detect, dictionary, estimate


def test_printed_marker_detection():
    d = dictionary('DICT_4X4_50')
    marker = cv2.aruco.drawMarker(d, 1, 200) if hasattr(cv2.aruco, 'drawMarker') else cv2.aruco.generateImageMarker(d, 1, 200)
    image = np.full((300, 300), 255, np.uint8)
    image[50:250, 50:250] = marker
    _, ids, _ = detect(image, d)
    assert ids.flatten().tolist() == [1]


def test_metric_pose_and_orientation():
    k = np.array([[800., 0, 320], [0, 800., 240], [0, 0, 1]])
    points = np.array([[-.025, .025, 0], [.025, .025, 0], [.025, -.025, 0], [-.025, -.025, 0]])
    expected_r = np.array([2.9, .1, .2])
    expected_t = np.array([.03, -.02, .45])
    corners, _ = cv2.projectPoints(points, expected_r, expected_t, k, np.zeros(5))
    r, t, error = estimate(corners, .05, k, np.zeros(5))
    np.testing.assert_allclose(t.flatten(), expected_t, atol=1e-6)
    np.testing.assert_allclose(Rotation.from_rotvec(r.flatten()).as_matrix(),
                               Rotation.from_rotvec(expected_r).as_matrix(), atol=1e-6)
    assert error < 1e-6


def test_ambiguous_branch_uses_continuity_but_rejects_large_reprojection_error():
    from so101_aruco.vision import choose_pose
    first = (np.array([[2.9], [.0], [.0]]), np.array([[0.], [0.], [.3]]), .1)
    second = (np.array([[3.2], [.0], [.0]]), np.array([[0.], [0.], [.3]]), .15)
    assert choose_pose([first, second], second[0], .3) is second
    bad_second = (second[0], second[1], 2.)
    assert choose_pose([first, bad_second], second[0], .3) is first
    assert choose_pose([first, second], None, .3) is first


def test_lm_refinement_reduces_error_on_noisy_corners():
    k = np.array([[810., 0, 320], [0, 811., 240], [0, 0, 1.]])
    points = np.array([[-.01, .01, 0], [.01, .01, 0], [.01, -.01, 0], [-.01, -.01, 0]])
    expected_r = np.array([2.9, .12, -.2])
    expected_t = np.array([.03, -.02, .23])
    corners, _ = cv2.projectPoints(points, expected_r, expected_t, k, np.zeros(5))
    corners += np.array([[[.2, -.1]], [[-.1, .15]], [[.12, .08]], [[-.15, -.2]]])
    raw = estimate(corners, .02, k, np.zeros(5), refine=False)
    refined = estimate(corners, .02, k, np.zeros(5), refine=True)
    assert refined[2] <= raw[2]
    assert refined[1][2, 0] > 0
