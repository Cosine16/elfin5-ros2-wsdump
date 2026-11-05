"""Transforms map child coordinates into parent coordinates; distances in metres."""
import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def transform(rotation=None, translation=(0, 0, 0)):
    result = np.eye(4)
    if rotation is not None:
        result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def from_msg(msg):
    t, q = msg.translation, msg.rotation
    return transform(Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(), [t.x, t.y, t.z])


def angle_deg(rotation):
    return float(np.rad2deg(Rotation.from_matrix(rotation).magnitude()))


def detect_marker(image, camera_matrix, distortion, marker_id=23, size=0.16):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    params = cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    corners, ids, _ = cv2.aruco.detectMarkers(image, dictionary, parameters=params)
    if ids is None or marker_id not in ids:
        raise ValueError(f'ArUco {marker_id} not visible')
    points = corners[list(ids.reshape(-1)).index(marker_id)].reshape(4, 2)
    half = size / 2
    object_points = np.array([[-half, half, 0], [half, half, 0],
                              [half, -half, 0], [-half, -half, 0]], dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(object_points, points, camera_matrix, distortion,
                                flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or tvec[2, 0] <= 0:
        raise ValueError('Invalid PnP solution')
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, distortion)
    error = float(np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - points) ** 2, axis=1))))
    if error > 1.0:
        raise ValueError(f'PnP reprojection error {error:.2f} px exceeds 1 px')
    return transform(cv2.Rodrigues(rvec)[0], tvec.reshape(3)), error, points


def solve(samples):
    if len(samples) < 10:
        raise ValueError('Need at least 10 distinct stationary observations')
    hands = [np.asarray(s['base_T_hand']) for s in samples]
    targets = [np.asarray(s['optical_T_target']) for s in samples]
    excitation = np.array([Rotation.from_matrix(hands[0][:3, :3].T @ h[:3, :3]).as_rotvec()
                           for h in hands[1:]])
    singular = np.linalg.svd(excitation, compute_uv=False)
    if singular[1] < 0.15:
        raise ValueError('Degenerate motion: rotate about at least two independent axes')
    rotation, translation = cv2.calibrateHandEye(
        [h[:3, :3] for h in hands], [h[:3, 3] for h in hands],
        [t[:3, :3] for t in targets], [t[:3, 3] for t in targets],
        method=cv2.CALIB_HAND_EYE_PARK)
    result = transform(rotation, translation.reshape(3))
    if not np.isfinite(result).all():
        raise ValueError('Non-finite calibration result')
    boards = [h @ result @ t for h, t in zip(hands, targets)]
    positions = np.array([b[:3, 3] for b in boards])
    mean_rotation = Rotation.from_matrix(np.array([b[:3, :3] for b in boards])).mean().as_matrix()
    metrics = {
        'board_translation_rms_mm': float(1000 * np.sqrt(np.mean(np.sum(
            (positions - positions.mean(axis=0)) ** 2, axis=1)))),
        'board_rotation_max_deg': max(angle_deg(mean_rotation.T @ b[:3, :3]) for b in boards),
        'rotation_excitation_singular_values_rad': singular.tolist(),
    }
    return result, metrics
