"""OpenCV-only detection and square-marker pose estimation. Lengths are metres."""
import cv2
import numpy as np


def dictionary(name):
    if not name.startswith('DICT_') or not hasattr(cv2.aruco, name):
        raise ValueError(f'Unknown ArUco dictionary: {name}')
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def detect(image, marker_dictionary, refinement_window=5, refinement_iterations=50, refinement_accuracy=0.001):
    # Use identical explicit subpixel settings on both old and new OpenCV APIs.
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    if hasattr(cv2.aruco, 'ArucoDetector'):
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
        corners, ids, rejected = cv2.aruco.ArucoDetector(marker_dictionary, parameters).detectMarkers(gray)
    else:
        parameters = cv2.aruco.DetectorParameters_create()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, marker_dictionary, parameters=parameters)
    for points in corners:
        cv2.cornerSubPix(gray, points.reshape(4, 2), (refinement_window, refinement_window), (-1, -1),
                         (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                          refinement_iterations, refinement_accuracy))
    return corners, ids, rejected


def estimate(corners, size, camera_matrix, distortion, previous_rotation=None, ambiguity_px=0.3, refine=True):
    half = size / 2.0
    points = np.array([[-half, half, 0], [half, half, 0],
                       [half, -half, 0], [-half, -half, 0]], dtype=np.float64)
    image_points = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    result = cv2.solvePnPGeneric(
        points, image_points, camera_matrix, distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    candidates = []
    for rotation, translation in zip(result[1], result[2]):
        if not np.isfinite(rotation).all() or not np.isfinite(translation).all() or translation[2, 0] <= 0:
            continue
        projected, _ = cv2.projectPoints(points, rotation, translation, camera_matrix, distortion)
        error = float(np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - image_points)**2, axis=1))))
        candidates.append((rotation, translation, error))
    selected = choose_pose(candidates, previous_rotation, ambiguity_px)
    if selected is None or not refine:
        return selected
    rotation, translation, error = selected
    refined_r, refined_t = cv2.solvePnPRefineLM(
        points, image_points, camera_matrix, distortion, rotation.copy(), translation.copy())
    if not np.isfinite(refined_r).all() or not np.isfinite(refined_t).all() or refined_t[2, 0] <= 0:
        return selected
    projected, _ = cv2.projectPoints(points, refined_r, refined_t, camera_matrix, distortion)
    refined_error = float(np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - image_points)**2, axis=1))))
    return (refined_r, refined_t, refined_error) if refined_error <= error + 1e-9 else selected


def choose_pose(candidates, previous_rotation=None, ambiguity_px=0.3):
    if not candidates:
        return None
    best = min(candidates, key=lambda item: item[2])
    if previous_rotation is None:
        return best
    previous_matrix, _ = cv2.Rodrigues(previous_rotation)
    plausible = [item for item in candidates if item[2] <= best[2] + ambiguity_px]
    def angle(item):
        matrix, _ = cv2.Rodrigues(item[0])
        return np.arccos(np.clip((np.trace(previous_matrix.T @ matrix) - 1) / 2, -1, 1))
    return min(plausible, key=angle)


class CornerHistory:
    """Short image-space median; reset on motion/gaps, never fill missing detections."""
    def __init__(self, window=5, motion_px=1.5, timeout_s=0.2):
        from collections import deque
        self.deque = deque
        self.window, self.motion_px, self.timeout_s = window, motion_px, timeout_s
        self.history = {}

    def update(self, marker_id, corners, stamp_s):
        points = np.asarray(corners, dtype=np.float32).reshape(4, 2)
        old = self.history.get(marker_id)
        reset = old is None
        if old is not None:
            last_stamp, queue = old
            reset = (stamp_s <= last_stamp or stamp_s-last_stamp > self.timeout_s or
                     np.max(np.linalg.norm(points-queue[-1], axis=1)) > self.motion_px or
                     np.max(np.linalg.norm(points-queue[0], axis=1)) > self.motion_px)
        if reset:
            queue = self.deque(maxlen=self.window)
        queue.append(points.copy())
        self.history[marker_id] = (stamp_s, queue)
        return np.median(np.stack(queue), axis=0).astype(np.float32).reshape(np.asarray(corners).shape)
