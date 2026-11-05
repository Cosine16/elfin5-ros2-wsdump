import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from elfin5_handeye.geometry import angle_deg, detect_marker, solve, transform


def samples_for(mount, rotations):
    rng = np.random.default_rng(42)
    board = transform(Rotation.from_euler('xyz', [.3, -.4, 1.2]).as_matrix(), [.75, 0, .55])
    samples = []
    for rotation in rotations:
        hand = transform(rotation, rng.uniform([.2, -.2, .3], [.5, .2, .7]))
        target = np.linalg.inv(mount) @ np.linalg.inv(hand) @ board
        samples.append({'base_T_hand': hand, 'optical_T_target': target})
    return samples


def test_handeye_recovers_child_to_parent_and_rejects_inverse_convention():
    mount = transform(Rotation.from_euler('xyz', [-1.2, .2, -.8]).as_matrix(), [.013, -.025, .07])
    rotations = Rotation.random(16, random_state=10).as_matrix()
    samples = samples_for(mount, rotations)
    estimated, metrics = solve(samples)
    np.testing.assert_allclose(estimated, mount, atol=1e-8)
    assert metrics['board_translation_rms_mm'] < 1e-6
    wrong = [dict(s, base_T_hand=np.linalg.inv(s['base_T_hand'])) for s in samples]
    _, wrong_metrics = solve(wrong)
    assert wrong_metrics['board_translation_rms_mm'] > 5


def test_degenerate_single_axis_and_insufficient_samples_are_rejected():
    samples = samples_for(np.eye(4), Rotation.from_euler('z', np.linspace(-.6, .6, 16)).as_matrix())
    with pytest.raises(ValueError, match='Degenerate'):
        solve(samples)
    with pytest.raises(ValueError, match='at least 10'):
        solve(samples[:3])


def test_rendered_world_marker_matches_detector_and_metric_size():
    # Reconstruct the actual SDF cell pattern, so ID, handedness and metre scale
    # are tested against the asset Gazebo consumes (not a second generated ID).
    sdf = ET.parse(Path(__file__).parents[1] / 'worlds/handeye.sdf')
    link = sdf.find(".//model[@name='aruco_6x6_250_id23']/link")
    marker = np.full((320, 320), 255, np.uint8)
    for visual in link.findall('visual'):
        if not visual.attrib['name'].startswith('cell_'):
            continue
        x, y, *_ = map(float, visual.findtext('pose').split())
        col, row = round(x / .02 + 3.5), round(3.5 - y / .02)
        marker[row * 40:(row + 1) * 40, col * 40:(col + 1) * 40] = 0
    image = np.full((480, 640), 255, np.uint8)
    matrix = np.array([[500., 0, 320], [0, 500., 240], [0, 0, 1.]])
    expected = transform(Rotation.from_euler('xyz', [np.pi - .2, .25, .1]).as_matrix(), [.03, -.02, .65])
    points = np.array([[-.08, .08, 0], [.08, .08, 0], [.08, -.08, 0], [-.08, -.08, 0]])
    projected, _ = cv2.projectPoints(points, cv2.Rodrigues(expected[:3, :3])[0], expected[:3, 3], matrix, None)
    homography = cv2.getPerspectiveTransform(np.float32([[0, 0], [319, 0], [319, 319], [0, 319]]),
                                           projected.astype(np.float32).reshape(4, 2))
    image = cv2.warpPerspective(marker, homography, (640, 480), borderValue=255)
    estimate, error, _ = detect_marker(image, matrix, np.zeros(5))
    assert np.linalg.norm(estimate[:3, 3] - expected[:3, 3]) < .006
    assert angle_deg(estimate[:3, :3].T @ expected[:3, :3]) < 2
    assert error < 1
    with pytest.raises(ValueError, match='not visible'):
        detect_marker(np.full_like(image, 255), matrix, np.zeros(5))
