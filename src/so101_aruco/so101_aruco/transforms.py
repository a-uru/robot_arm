"""Rigid transforms: A_T_B maps B coordinates to A coordinates."""
import numpy as np
from scipy.spatial.transform import Rotation


def from_xyz_rpy(xyz, rpy_deg):
    if len(xyz) != 3 or len(rpy_deg) != 3 or not np.isfinite([xyz, rpy_deg]).all():
        raise ValueError('Offsets require three finite xyz values and three finite RPY values')
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler('xyz', rpy_deg, degrees=True).as_matrix()
    transform[:3, 3] = xyz
    return transform


def from_pose(pose):
    transform = np.eye(4)
    q = pose.orientation
    transform[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    p = pose.position
    transform[:3, 3] = [p.x, p.y, p.z]
    return transform


def base_object(base_reference, camera_reference, camera_object, marker_object):
    return base_reference @ np.linalg.inv(camera_reference) @ camera_object @ marker_object


def camera_link_to_optical():
    """Optical X right/Y down/Z forward expressed in camera_link X forward/Y left/Z up."""
    transform = np.eye(4)
    transform[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
    return transform
