"""Camera/registration/inference diagnostic ONLY. No motor interface imported.

Policy snapshots assume DEFAULT_POSE and zero tracking error; neither encoder
feedback nor physical simulation. Never report manipulation success from this.
"""
import json
import sys
import time
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parent / 'work/Leap_Test'
sys.path.insert(0, str(PROJECT / 'deploy'))
import limits
import obs_builder as ob
from policy_np import Policy
from hand_registration_geometry import candidate


def infer_snapshot(policy, pose):
    """Independent hypothetical snapshot; no accumulated motor commands."""
    if pose is None or pose.get('n_cube_markers', 0) < 2:
        return None
    if not all(k in pose for k in ('palm_minus_cube', 'cube_quat_world', 'reproj')):
        return None
    p = np.asarray(pose['palm_minus_cube'], float)
    q = np.asarray(pose['cube_quat_world'], float)
    if p.shape != (3,) or q.shape != (4,) or not np.isfinite(np.r_[p, q, pose['reproj']]).all():
        return None
    if pose['reproj'] > 3.0 or not np.isclose(np.linalg.norm(q), 1, atol=.01):
        return None
    builder = ob.ObsBuilder()
    assumed_q = limits.DEFAULT_POSE.copy()
    # Fill a stationary history, with no fictitious actuation feedback.
    for _ in range(ob.HISTORY_LEN):
        obs = builder.build(assumed_q, assumed_q, p, np.zeros(3), q, ob.FACE_GOAL_QUATS[0])
    act = np.asarray(policy(obs), float)
    if obs.shape != (108,) or act.shape != (16,) or not np.isfinite(np.r_[obs, act]).all():
        raise ValueError('Invalid policy observation/action')
    if np.max(np.abs(act)) > 1.000001:
        raise ValueError('Policy output out of range')
    return {'palm_minus_cube_m': p.tolist(), 'cube_quat_world': q.tolist(),
            'observation': obs.tolist(), 'action': act.tolist(),
            'unclipped_hypothetical_delta_rad': (limits.ACTION_SCALE * act).tolist(),
            'reprojection_px': float(pose['reproj'])}


def main():
    import cv2 as cv
    from camera_720_check import open_hd
    sys.path.insert(0, str(PROJECT / 'vision/rubik_center'))
    import cube_board as cb
    import track_cube as tc
    registration = candidate()
    # Process-local diagnostic settings only; source/deployment defaults untouched.
    tc.PALM_IN_MARKER = np.array(registration['grasp_site_in_marker_m_candidate'])
    tc.HAND_TO_WORLD = np.array(registration['hand_to_world_candidate'])
    cal = np.load(ROOT / 'camera1_720.npz')
    if (int(cal['height']), int(cal['width'])) != (720, 1280):
        raise ValueError('Expected 1280x720 calibration')
    K, dist = cal['K'], cal['dist']
    rotations = tuple(json.loads((ROOT / 'cube_face_rot_result.json').read_text())['face_rot'])
    points, ids = cb.object_points(cube_mm=54.5, marker_mm=10.6, layout='centers', face_rot=rotations)
    dictionary, _, detector = tc.build(marker_mm=10.6, grid=1)
    board = cv.aruco.Board(points, dictionary, ids)
    policy = Policy(PROJECT / 'policy/policy_np.npz')
    cap, capture, _ = open_hd(cv, 1, require_mjpg=True)
    samples, frames, failed_reads = [], 0, 0
    start = None
    last_frame = time.monotonic()
    title = 'REGISTERED POLICY CHECK - NO MOTORS'
    cv.namedWindow(title, cv.WINDOW_NORMAL)
    cv.resizeWindow(title, 1100, 620)
    print('NO MOTORS. Encoder angles NOT read: hypothetical DEFAULT_POSE only.', flush=True)
    print('Keep cube still on hand; show ID40 and 2 cube markers. SPACE: 10 seconds; S: photo; Q: quit.', flush=True)
    print('Magenta: model grasp reference (not marker center); green: detected cube center.', flush=True)
    completed = False
    error = None
    try:
        while True:
            t0 = time.monotonic()
            if start is not None and t0 - start >= 10:
                completed = True
                break
            ok, frame = cap.read()
            if not ok or frame is None:
                failed_reads += 1
                if time.monotonic() - last_frame > 5:
                    raise RuntimeError('No camera frame for 5 seconds')
                if cv.waitKey(1) & 255 in (ord('q'), 27):
                    break
                continue
            last_frame = time.monotonic()
            if frame.shape[:2] != (720, 1280):
                raise RuntimeError('Camera resolution changed')
            pose = tc.step(frame, board, detector, dictionary, K, dist)
            snapshot = infer_snapshot(policy, pose)
            if pose and 't_hand' in pose:
                origin = pose['t_hand'] + pose['R_hand'] @ tc.PALM_IN_MARKER
                targets = [('grasp ref', origin, (255, 0, 255))]
                if snapshot is not None:
                    targets.append(('cube', pose['t_cube'], (0, 255, 0)))
                for label, point, color in targets:
                    if np.isfinite(point).all() and point[2] > 0:
                        xy = cv.projectPoints(np.array([point]), np.zeros(3), np.zeros(3), K, dist)[0].reshape(2)
                        if np.isfinite(xy).all() and 0 <= xy[0] < 1280 and 0 <= xy[1] < 720:
                            at = tuple(np.rint(xy).astype(int))
                            cv.drawMarker(frame, at, color, cv.MARKER_CROSS, 20, 2)
                            cv.putText(frame, label, (at[0]+12, at[1]), cv.FONT_HERSHEY_SIMPLEX, .6, color, 2)
            n = pose.get('n_cube_markers', 0) if pose else 0
            lines = ['NO MOTORS | UNVERIFIED DEPTH | ASSUMED JOINT ANGLES',
                     f'Cube markers: {n} | inference: {"YES (hypothetical)" if snapshot else "SKIPPED"}',
                     'Goal: white face up | SPACE 10s | S photo | Q quit']
            if snapshot:
                mm = np.array(snapshot['palm_minus_cube_m']) * 1000
                lines.append('grasp-cube [model XYZ mm]: ' + ', '.join(f'{x:.1f}' for x in mm))
                lines.append(f"Hypothetical max |delta|: {max(abs(x) for x in snapshot['unclipped_hypothetical_delta_rad']):.3f} rad (NOT sent)")
            if start is not None:
                frames += 1
                if snapshot:
                    samples.append(dict(snapshot, time_s=time.monotonic()-start,
                                        cube_markers=n, processing_ms=(time.monotonic()-t0)*1000))
                lines.append(f'{max(0, 10-(time.monotonic()-start)):.1f}s left | eligible {len(samples)}/{frames}')
            for i, line in enumerate(lines):
                cv.putText(frame, line, (10, 25+i*25), cv.FONT_HERSHEY_SIMPLEX, .55, (0, 255, 255), 2)
            cv.imshow(title, frame)
            key = cv.waitKey(1) & 255
            if key == ord('s'):
                ok, encoded = cv.imencode('.png', frame)
                if not ok:
                    raise RuntimeError('Photo encoding failed')
                (ROOT / 'registered_policy_check.png').write_bytes(encoded.tobytes())
                print('Saved registered_policy_check.png', flush=True)
            if key in (ord('q'), 27):
                break
            if key == 32 and start is None:
                start = time.monotonic()
                print('Measuring 10 seconds; no servo commands.', flush=True)
    except Exception as exc:
        error = str(exc)
        raise
    finally:
        cap.release()
        cv.destroyAllWindows()
        report = {'status': 'error' if error else ('complete' if completed else 'stopped'),
                  'error': error, 'motors_accessed': False, 'hardware_authorized': False,
                  'joint_source': 'hypothetical DEFAULT_POSE, zero tracking error; NOT encoder feedback',
                  'not_a_simulation_or_control_success_test': True,
                  'registration': registration, 'capture': capture, 'face_rot': rotations,
                  'frames': frames, 'eligible_frames': len(samples), 'read_failures': failed_reads,
                  'eligible_percent': 100*len(samples)/frames if frames else None,
                  'diagnostic_gate': '>=2 cube markers, hand pose, reprojection <=3px, finite pose/action',
                  'samples': samples}
        path = ROOT / 'registered_policy_check_result.json'
        path.write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(f'Saved: {path}', flush=True)
        print(f'Eligible {len(samples)}/{frames}; NO MOTORS; not hardware approval.', flush=True)


if __name__ == '__main__':
    main()
