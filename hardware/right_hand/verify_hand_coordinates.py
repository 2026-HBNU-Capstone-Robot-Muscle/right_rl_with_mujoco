"""Offline coordinate audit. No camera, serial port, or motor access.

Synthetic image points test the production PnP -> world -> observation path.
Algebraic agreement does NOT verify physical marker mounting or dimensions.
"""
import datetime
import json
import sys
from pathlib import Path
import cv2 as cv
import numpy as np
from hand_registration_geometry import candidate

ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parent/'work/Leap_Test'
sys.path.insert(0, str(PROJECT/'vision/rubik_center'))
sys.path.insert(0, str(PROJECT/'deploy'))
import cube_board as cb
import track_cube as tc
from obs_builder import ObsBuilder, quat_to_mat


def main():
    reg = candidate()
    R_wm = np.array(reg['hand_to_world_candidate'])
    grasp_m = np.array(reg['grasp_site_in_marker_m_candidate'])
    tc.HAND_TO_WORLD, tc.PALM_IN_MARKER = R_wm, grasp_m
    np.testing.assert_allclose(R_wm.T@R_wm, np.eye(3), atol=1e-12)
    np.testing.assert_allclose(np.linalg.det(R_wm), 1., atol=1e-12)
    with np.load(ROOT/'camera1_720.npz') as cal:
        K, dist = cal['K'].copy(), cal['dist'].copy()
        assert (int(cal['width']), int(cal['height'])) == (1280, 720)
    assert np.isfinite(K).all() and np.isfinite(dist).all()
    assert K[0,0]>0 and K[1,1]>0
    rotations = tuple(json.loads((ROOT/'cube_face_rot_result.json').read_text())['face_rot'])
    points, ids = cb.object_points(cube_mm=54.5, marker_mm=10.6,
                                  layout='centers', face_rot=rotations)
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    board = cv.aruco.Board(points, dictionary, ids)
    h = tc.HAND_MM/2000
    hand = np.array([[-h,h,0], [h,h,0], [h,-h,0], [-h,-h,0]], np.float64)
    cube_m = np.array([.02, .08, .10])
    R_mc = cv.Rodrigues(np.array([.2, -.35, .1]))[0]
    expected_d = R_wm@(grasp_m-cube_m)
    expected_R = R_wm@R_mc
    checks = []
    for rv in ([.3,.1,-.1], [-.15,.4,.2], [.5,-.2,-.3]):
        R_ch = cv.Rodrigues(np.array(rv))[0]
        t_ch = np.array([.015, -.025, .6])
        R_cc, t_cc = R_ch@R_mc, t_ch+R_ch@cube_m
        def project(obj, R, t):
            return cv.projectPoints(np.asarray(obj, dtype=np.float64),
                cv.Rodrigues(R)[0], t, K, dist)[0].reshape(1,4,2).astype(np.float32)
        # Two adjacent cube faces: exact numeric points, not visibility evidence.
        corners = [project(points[i], R_cc, t_cc) for i in (0,2)]
        corners.append(project(hand, R_ch, t_ch))
        detected = np.array([int(ids[0]), int(ids[2]), tc.HAND_ID], np.int32).reshape(-1,1)
        class Detector:
            def detectMarkers(self, gray): return corners, detected, []
        pose = tc.step(np.zeros((1,1,3), np.uint8), board, Detector(), dictionary, K, dist)
        np.testing.assert_allclose(pose['palm_minus_cube'], expected_d, atol=1e-5)
        np.testing.assert_allclose(pose['R_cube_world'], expected_R, atol=1e-5)
        np.testing.assert_allclose(quat_to_mat(pose['cube_quat_world']), expected_R, atol=1e-5)
        obs = ObsBuilder().build(np.zeros(16), np.zeros(16),
            pose['palm_minus_cube'], np.zeros(3), pose['cube_quat_world'], np.array([1.,0,0,0]))
        assert obs.shape == (108,)
        np.testing.assert_allclose(obs[64:67], expected_d, atol=1e-5)
        np.testing.assert_allclose(obs[73:79], expected_R.ravel()[3:], atol=1e-5)
        checks.append(dict(camera_rotation_vector=rv,
            position_error_mm=float(np.linalg.norm(pose['palm_minus_cube']-expected_d)*1000),
            reprojection_px=float(pose['reproj'])))
    # Exercise matrix-to-wxyz conversion at 180 degrees around every axis too.
    for axis in np.eye(3):
        R = cv.Rodrigues(axis*np.pi)[0]
        np.testing.assert_allclose(quat_to_mat(tc.mat_to_quat(R)), R, atol=1e-12)
    latest = json.loads((ROOT/'leap_resume_result.json').read_text(encoding='utf-8'))
    saved_reg = latest.get('registration')
    if saved_reg:
        np.testing.assert_allclose(saved_reg['hand_to_world_candidate'], R_wm, atol=1e-12)
        np.testing.assert_allclose(saved_reg['grasp_site_in_marker_m_candidate'], grasp_m, atol=1e-12)
    report = dict(timestamp=datetime.datetime.now().astimezone().isoformat(),
        status='algebra_pass_physical_registration_unverified', motor_access=False,
        registration=reg, synthetic_camera_cases=checks,
        verified=['rotation orthonormality/right-handedness', 'camera-view invariance',
            'grasp-minus-cube sign and metres', 'wxyz including 180-degree rotations',
            '108D observation position/orientation slices'],
        latest_run_timestamp=latest.get('timestamp'), latest_run_registration_matches=bool(saved_reg),
        hand_marker_size_mm=tc.HAND_MM,
        remaining=['physical marker edge length (currently 28.2mm provisional)',
            'marker-plane tilt/yaw and motor-axis landmark alignment',
            'provisional depth: motor 9 axis 14mm below marker plane',
            'live cube pose stability: successful trial did not store pose values'])
    path = ROOT/'hand_coordinate_audit_result.json'
    path.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print('PASS: 3 synthetic camera views, 108D sign/rotation path, 180-degree quaternion cases.')
    print('Physical registration remains unverified. Saved:', path.name)


if __name__ == '__main__': main()
