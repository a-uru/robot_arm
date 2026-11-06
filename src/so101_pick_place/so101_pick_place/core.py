"""Pure validation and sequence generation; all positions refer to the TCP."""
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class Stage:
    name: str
    xyz: object = None
    gripper: object = None
    carrying: bool = False


def vector(value, name):
    a = np.asarray(value, dtype=float)
    if a.shape != (3,) or not np.isfinite(a).all():
        raise ValueError(f'{name} must contain three finite values')
    return a


def grasp_point(position, quaternion, offset):
    return vector(position, 'object position') + Rotation.from_quat(quaternion).apply(vector(offset, 'grasp offset'))


def stages(pick, place, p):
    pick = vector(pick, 'pick'); place = vector(place, 'place')
    z = np.array([0., 0., 1.])
    clearance = pick.copy()
    clearance[2] = max(pick[2], float(p['transport_clearance_z_m']))
    return [Stage('open', gripper=p['gripper_open_rad']),
            Stage('approach', pick + z * p['approach_height_m']),
            Stage('grasp', pick),
            Stage('close', gripper=p['gripper_closed_rad']),
            Stage('clearance', clearance, carrying=True),
            Stage('place', place, carrying=True),
            Stage('release', gripper=p['gripper_open_rad'], carrying=True),
            Stage('wait', vector(p['wait_xyz_m'], 'wait_xyz_m'))]


def validate_stages(sequence, lower, upper):
    lower = vector(lower, 'workspace_min_m'); upper = vector(upper, 'workspace_max_m')
    if np.any(lower >= upper):
        raise ValueError('workspace min must be below max')
    for stage in sequence:
        if stage.xyz is not None and (np.any(stage.xyz < lower) or np.any(stage.xyz > upper)):
            raise ValueError(f'{stage.name}: TCP target {stage.xyz.tolist()} is outside workspace')


def stable_snapshot(samples, now, p):
    """samples = (monotonic receipt time, ROS stamp seconds, xyz, quaternion)."""
    if not samples or now - samples[-1][0] > p['target_timeout_s']:
        raise ValueError('No fresh same-image ID0/ID1 pose; show both markers')
    recent = [s for s in samples if now - s[0] <= p['stable_window_s']]
    if len(recent) < p['stable_min_samples'] or recent[-1][0] - recent[0][0] < p['stable_window_s'] * .7:
        raise ValueError('Wait for a stable observation window')
    if len({s[1] for s in recent}) < p['stable_min_samples']:
        raise ValueError('Pose timestamps are not advancing')
    if np.max(np.diff([s[0] for s in recent])) > p['target_timeout_s']:
        raise ValueError('Observation window contains a detection gap')
    positions = np.array([grasp_point(s[2], s[3], p['grasp_offset_object_m']) for s in recent])
    if np.max(np.std(positions, axis=0)) > p['stable_position_std_m'] or np.max(np.ptp(positions, axis=0)) > p['stable_position_range_m']:
        raise ValueError('Target estimate is moving/unstable; keep markers still')
    # Only freeze the most recent measured target, never a stale TF lookup.
    return positions[-1].copy()
