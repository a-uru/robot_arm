from types import SimpleNamespace
import numpy as np
from so101_aruco.node import ArucoDetector


def fake_node(valid):
    sent, published = [], []
    node = SimpleNamespace(reference_id=0, object_id=1, base_frame='base_link', base_prediction_frame='base_link_predict',
                           object_frame='aruco_object_origin', offset_valid=valid,
                           publish_camera_tf=True, camera_link_frame="camera_link", static_optical_frame=None, base_reference=np.eye(4), marker_object=np.eye(4),
                           send_matrix=lambda *args: sent.append(args),
                           object_pose=SimpleNamespace(publish=published.append),
                           get_logger=lambda: SimpleNamespace(warning=lambda *a, **kw: None))
    return node, sent, published


def test_uncalibrated_outputs_preview_but_no_base_pose_or_camera_tf():
    node, sent, published = fake_node(False)
    ArucoDetector.publish_offsets(node, SimpleNamespace(frame_id='camera_optical_frame', stamp=None),
                                  {0: np.eye(4), 1: np.eye(4)})
    assert not published
    assert {v[2] for v in sent} == {'base_link_predict', 'aruco_object_origin'}


def test_missing_reference_never_reuses_previous_observation():
    node, sent, published = fake_node(True)
    ArucoDetector.publish_offsets(node, SimpleNamespace(frame_id='camera_optical_frame', stamp=None),
                                  {1: np.eye(4)})
    assert not published
    assert all(v[1] != 'base_link' for v in sent)


def test_reference_alone_updates_camera_but_not_object_pose():
    node, sent, published = fake_node(True)
    ArucoDetector.publish_offsets(node, SimpleNamespace(frame_id='camera_optical_frame', stamp=None),
                                  {0: np.eye(4)})
    assert not published
    assert any(v[1] == 'base_link' and v[2] == 'camera_link' for v in sent)
