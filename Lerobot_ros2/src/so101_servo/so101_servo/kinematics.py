"""URDF serial-chain FK and translational Jacobian (radians/metres)."""
import math
import xml.etree.ElementTree as ET
import numpy as np


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm <= 0:
        raise ValueError("Joint axis must be non-zero")
    x, y, z = axis / norm
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle)*cross + (1-math.cos(angle))*(cross@cross)


def origin_transform(element):
    out = np.eye(4)
    if element is not None:
        out[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
        r, p, y = np.fromstring(element.get("rpy", "0 0 0"), sep=" ")
        out[:3, :3] = rotation([0, 0, 1], y) @ rotation([0, 1, 0], p) @ rotation([1, 0, 0], r)
    return out


class Chain:
    def __init__(self, description, base, tip):
        robot = ET.fromstring(description)
        joints = {j.find("child").get("link"): j for j in robot.findall("joint")}
        chain = []
        link = tip
        visited = set()
        while link != base:
            if link in visited or link not in joints:
                raise ValueError(f"No serial chain from {base} to {tip}")
            visited.add(link)
            j = joints[link]
            chain.append(j)
            link = j.find("parent").get("link")
        self.chain = list(reversed(chain))
        self.names = [j.get("name") for j in self.chain if j.get("type") != "fixed"]
        if any(j.get("type") not in ("fixed", "revolute", "continuous") for j in self.chain):
            raise ValueError("Only fixed and revolute joints are supported")

    def evaluate(self, positions, offset=None):
        transform = np.eye(4)
        origins, axes = [], []
        for j in self.chain:
            transform = transform @ origin_transform(j.find("origin"))
            if j.get("type") != "fixed":
                element = j.find("axis")
                axis = np.fromstring(element.get("xyz", "1 0 0") if element is not None else "1 0 0", sep=" ")
                axis /= np.linalg.norm(axis)
                origins.append(transform[:3, 3].copy())
                axes.append(transform[:3, :3] @ axis)
                turn = np.eye(4)
                turn[:3, :3] = rotation(axis, positions[j.get("name")])
                transform = transform @ turn
        offset = np.zeros(3) if offset is None else np.asarray(offset, dtype=float)
        if offset.shape != (3,) or not np.all(np.isfinite(offset)):
            raise ValueError("Tool offset must contain three finite values")
        point = transform[:3, 3] + transform[:3, :3] @ offset
        jacobian = np.column_stack([np.cross(a, point-o) for a, o in zip(axes, origins)])
        return point, jacobian


def tracking_velocity(error, jacobian, gain, max_linear_speed, damping, max_joint_speed):
    error = np.asarray(error, dtype=float)
    if not np.all(np.isfinite(error)) or not np.all(np.isfinite(jacobian)):
        raise ValueError("Non-finite tracking input")
    velocity = gain * error
    speed = np.linalg.norm(velocity)
    if speed > max_linear_speed:
        velocity *= max_linear_speed/speed
    joint_velocity = jacobian.T @ np.linalg.solve(
        jacobian @ jacobian.T + damping*damping*np.eye(3), velocity)
    peak = np.max(np.abs(joint_velocity))
    if peak > max_joint_speed:
        joint_velocity *= max_joint_speed/peak
    return joint_velocity


def offset_point(position, quaternion, offset):
    """Transform a local tool offset into the base frame using a TF pose."""
    q = np.asarray(quaternion, dtype=float)
    offset = np.asarray(offset, dtype=float)
    if q.shape != (4,) or offset.shape != (3,) or not np.all(np.isfinite(q)) or not np.all(np.isfinite(offset)):
        raise ValueError("Invalid tool pose/offset")
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        raise ValueError("Invalid tool rotation")
    q = q / norm
    a, w = q[:3], q[3]
    return np.asarray(position) + offset + 2*np.cross(a, np.cross(a, offset)+w*offset)
