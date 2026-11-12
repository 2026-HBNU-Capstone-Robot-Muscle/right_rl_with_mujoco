"""Offline parser check using existing camera frames plus a synthetic fixed ID41."""
import tempfile
import sys
import threading
import time
from types import SimpleNamespace
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from sac_registration_from_anchor import estimate
from sequence_vision_recorder import SequenceVisionRecorder


def load(path):
    return cv2.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv2.IMREAD_COLOR)


def save(path, frame):
    cv2.imencode('.png', frame)[1].tofile(str(path))


def test_two_frame_anchor_parser():
    folder = ROOT / 'sequence_vision_20261006_175952_075873'
    empty = load(folder / 'stage_0_not_ready.png')
    placed = load(folder / 'failure_nearest.png')
    marker = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 41, 100)
    white = cv2.copyMakeBorder(marker, 12, 12, 12, 12, cv2.BORDER_CONSTANT, value=255)
    patch = cv2.cvtColor(white, cv2.COLOR_GRAY2BGR)
    for frame in (empty, placed):
        frame[60:184, 1050:1174] = patch
    with tempfile.TemporaryDirectory() as name:
        path = Path(name)
        save(path / 'anchor_empty_stage0.png', empty)
        save(path / 'placement_at_space.png', placed)
        result = estimate(path)
    assert result['status'] == 'anchor_pair_candidate'
    assert 40 in result['empty_ids'] and 41 in result['empty_ids']
    assert 41 in result['placed_ids']
    assert len([i for i in result['placed_ids'] if i < 40]) >= 2
    assert np.isfinite(result['cube_center_in_sim_world_candidate_m']).all()


def test_single_cube_marker_parser():
    folder = ROOT / 'sequence_vision_20261006_175952_075873'
    empty = load(folder / 'stage_0_not_ready.png')
    placed = load(folder / 'failure_nearest.png')
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(cv2.cvtColor(placed, cv2.COLOR_BGR2GRAY))
    remove = next(np.asarray(c).reshape(4, 2) for c, i in zip(corners, ids.ravel()) if int(i) == 1)
    cv2.fillConvexPoly(placed, np.rint(remove).astype(np.int32), (255, 255, 255))
    marker = cv2.aruco.generateImageMarker(dictionary, 41, 100)
    patch = cv2.cvtColor(cv2.copyMakeBorder(marker, 12, 12, 12, 12,
                                            cv2.BORDER_CONSTANT, value=255), cv2.COLOR_GRAY2BGR)
    for frame in (empty, placed):
        frame[60:184, 1050:1174] = patch
    with tempfile.TemporaryDirectory() as name:
        path = Path(name)
        save(path / 'anchor_empty_stage0.png', empty)
        save(path / 'placement_at_space.png', placed)
        result = estimate(path)
    assert result['status'] == 'anchor_single_marker_candidate'
    assert result['cube_marker_count'] == 1
    assert np.isfinite(result['cube_center_in_sim_world_candidate_m']).all()


def test_anchor_rigidity_gate():
    folder = ROOT / 'sequence_vision_20261006_195816_286954'
    preflight = load(folder / 'preflight_not_ready.png')
    stage0 = load(folder / 'anchor_empty_stage0.png')
    recorder = SequenceVisionRecorder.__new__(SequenceVisionRecorder)
    recorder.anchor_mode = True
    recorder.frame_lock = threading.Lock()
    recorder.camera = SimpleNamespace(cv=cv2)
    recorder.root = ROOT
    recorder.report = {}
    recorder.anchor_preflight_frame = preflight
    recorder.anchor_empty_frame = preflight.copy()
    assert recorder.check_anchor_rigidity() < 1.0
    recorder.anchor_empty_frame = stage0
    try:
        recorder.check_anchor_rigidity()
    except RuntimeError as exc:
        assert 'reference moved/warped' in str(exc)
        assert recorder.report['anchor_rigid_projection_error_px'] > 70
    else:
        raise AssertionError('Known moving-reference capture was accepted')


def test_stale_candidate_exits_with_diagnostic():
    recorder = SequenceVisionRecorder.__new__(SequenceVisionRecorder)
    recorder.frame_lock = threading.Lock()
    recorder.anchor_empty_frame = np.zeros((16, 16, 3), np.uint8)
    row = {'detected_ids': [5], 'anchor_detected_ids': [41], 'reason': 'cube_markers_below_2'}
    recorder.anchor_cube_candidate = (time.monotonic() - 10, np.zeros((16, 16, 3), np.uint8), row)
    recorder.last_analyzed = None
    result = recorder.mark_placement(require_anchor_cube=True)
    assert result['capture_stale'] is True
    assert result['capture_age_s'] >= 10
    assert recorder.placement_frame is not None


if __name__ == '__main__':
    test_two_frame_anchor_parser()
    test_single_cube_marker_parser()
    test_anchor_rigidity_gate()
    test_stale_candidate_exits_with_diagnostic()
    print('Anchor two-frame parser self-check passed.')
