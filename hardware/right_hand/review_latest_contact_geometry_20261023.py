"""Direct same-image palm reference and full cube rim; offline diagnostic only.

No old wrist bridge, hidden cube corner, hand offset fitting or motor export.
The manual visible rim pixels have explicit uncertainty; the marker/model
mount and camera intrinsics remain provisional.
"""
import itertools
import json
from pathlib import Path

import cv2 as cv
import numpy as np

from hand_registration_geometry import candidate
from marker_detector import MarkerDetector
from review_prepared_cube_20261022 import marker_solutions

ROOT = Path(__file__).resolve().parent
FOLDER = ROOT / 'cube_grasp_trial_20261023_044607_477240'


def read_image(path):
    image = cv.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv.IMREAD_COLOR)
    if image is None:
        raise RuntimeError('Saved image is unreadable: ' + str(path))
    return image


def cube_fit(pixels, K, distortion):
    s = .055 / 2
    # Top back-left, back-right, front-right, front-left, visible front-bottom-left.
    points = np.array([[-s,s,s],[s,s,s],[s,-s,s],[-s,-s,s],[-s,-s,-s]], np.float64)
    ok, rv, tv = cv.solvePnP(points, pixels, K, distortion, flags=cv.SOLVEPNP_SQPNP)
    if not ok:
        raise RuntimeError('No full-rim cube fit')
    rv, tv = cv.solvePnPRefineLM(points, pixels, K, distortion, rv, tv)
    R = cv.Rodrigues(rv)[0]
    if np.min((points @ R.T + tv.ravel())[:,2]) <= 0:
        raise RuntimeError('Cube fit behind camera')
    projected = cv.projectPoints(points, rv, tv, K, distortion)[0].reshape(-1,2)
    return R, tv.ravel(), float(np.mean(np.linalg.norm(projected - pixels, axis=1)))


def main():
    trial = json.loads((FOLDER / 'result.json').read_bytes())
    frame_metadata = json.loads((FOLDER / 'camera' / 'frames.json').read_bytes())['frames']
    camera_time = trial['grasp_observation_photo']['camera_read_monotonic']
    metadata = min(frame_metadata, key=lambda f: abs(f['camera_read_monotonic'] - camera_time))
    stamp = camera_time - trial['enabled_at']
    sample = min(trial['samples'], key=lambda row: abs(row['t'] - stamp))
    image = read_image(FOLDER / 'cube_after_approach.jpg')
    with np.load(ROOT / 'camera1_720.npz') as data:
        K, distortion = data['K'], data['dist']
    detector = MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),
                              range(6), hand_id=40, required_cube_count=1)
    corners, ids, _ = detector.detectMarkers(cv.cvtColor(image, cv.COLOR_BGR2GRAY))
    ids = [] if ids is None else list(map(int, ids.ravel()))
    points = {str(ident): np.asarray(c).reshape(4,2).tolist() for ident,c in zip(ids,corners)}
    palm_poses = marker_solutions(corners, ids, 40, .0282, K, distortion)
    palm_reference_source = 'decoded_ID40'
    manual_palm_pixels = None
    if not palm_poses:
        # Glare prevents decoding ID40 in this recording. Retain that failure:
        # visible border geometry can be explored, never labeled valid vision.
        manual_palm_pixels = [[442,414],[560,416],[560,508],[430,507]]
        palm_poses = marker_solutions([np.array(manual_palm_pixels)], [40], 40, .0282, K, distortion)
        palm_reference_source = 'manual_known_palm_border_not_decoded_or_verified'

    # Rounded plastic corners are not ideal cube vertices. Read straight edge
    # segments independently from the photo, then intersect their lines; do
    # not move corner coordinates to minimize the subsequent pose residual.
    edge_segments = dict(
        back_top=[[612,141],[814,136]],
        left_top=[[589,167],[596,234]],
        right_top=[[846,158],[881,226]],
        front_top=[[619,249],[865,245]],
        front_left_vertical=[[597,278],[604,447]],
        front_bottom=[[620,472],[696,471]],
    )
    def intersection(first,second):
        a,b=np.c_[np.asarray(edge_segments[first],float),np.ones(2)]
        c,e=np.c_[np.asarray(edge_segments[second],float),np.ones(2)]
        value=np.cross(np.cross(a,b),np.cross(c,e))
        return value[:2]/value[2]
    visible_rim = np.array([
        intersection('back_top','left_top'),intersection('back_top','right_top'),
        intersection('front_top','right_top'),intersection('front_top','left_top'),
        intersection('front_left_vertical','front_bottom'),
    ])
    cube_R, cube_t, rim_error = cube_fit(visible_rim, K, distortion)
    registration = candidate()
    R_wm = np.asarray(registration['hand_to_world_candidate'])
    grasp_m = np.asarray(registration['grasp_site_in_marker_m_candidate'])
    grasp_w = np.array([.11,0,.03])
    hypotheses = []
    rng = np.random.default_rng(20261023)
    jittered = []
    # +/-3px per-coordinate image uncertainty, not a mounting/scale posterior.
    for _ in range(128):
        jittered.append(cube_fit(visible_rim+rng.uniform(-3,3,visible_rim.shape), K, distortion))
    for index, (R_cm, t_cm, marker_error) in enumerate(palm_poses):
        R_cw = R_cm @ R_wm.T
        t_cw = t_cm + R_cm @ grasp_m - R_cw @ grasp_w
        cube_world = R_cw.T @ (cube_t-t_cw)
        cube_world_R = R_cw.T @ cube_R
        uncertainty = np.array([R_cw.T @ (j[1]-t_cw) for j in jittered])
        hypotheses.append(dict(
            palm_ippe_branch=index, marker_error_px=marker_error,
            world_to_camera_rotation=R_cw.tolist(), world_to_camera_translation=t_cw.tolist(),
            cube_center_world_m=cube_world.tolist(), cube_rotation_world=cube_world_R.tolist(),
            cube_center_marker40_m=(R_cm.T@(cube_t-t_cm)).tolist(),
            rim_pixel_uncertainty_center_min_world_m=uncertainty.min(axis=0).tolist(),
            rim_pixel_uncertainty_center_max_world_m=uncertainty.max(axis=0).tolist(),
        ))

    # Independently decoded sticker poses remain diagnostics, not the fitting target.
    sticker_rows = []
    for R_c5,t_c5,error in marker_solutions(corners, ids, 5, .0107, K, distortion):
        marker_center = t_c5
        top_center = cube_t+cube_R[:,2]*(.055/2)
        sticker_rows.append(dict(error_px=error, marker_center_camera_m=marker_center.tolist(),
                                 distance_to_rim_top_center_mm=float(np.linalg.norm(marker_center-top_center)*1000)))

    selection = []
    for f in frame_metadata:
        if f['phase']=='remove_cube' and int(Path(f['image']).stem.split('_')[-1]) % 12 == 0:
            selection.append(f)
    side_detection_rows = []
    for f in selection:
        im = read_image(FOLDER / 'camera' / f['image'])
        cs, found, _ = detector.detectMarkers(cv.cvtColor(im, cv.COLOR_BGR2GRAY))
        found = [] if found is None else list(map(int,found.ravel()))
        side_detection_rows.append(dict(image=f['image'], ids=found,
            corners={str(i):np.asarray(c).reshape(4,2).tolist() for i,c in zip(found,cs)}))

    report = dict(
        status='direct_same_image_palm_cube_rim_diagnostic', source=FOLDER.name,
        image='cube_after_approach.jpg', nearest_source_frame=metadata['image'],
        camera_read_monotonic=camera_time, sample_t=sample['t'], camera_sample_dt_s=sample['t']-stamp,
        measured_motor_ticks=sample['actual'], goal_ticks=sample['goal'],
        cube_side_mm=55, palm_marker_side_mm=28.2,
        detected_ids=ids, marker_corners=points, visible_rim_pixels=visible_rim.tolist(),
        palm_reference_source=palm_reference_source, manual_palm_pixels=manual_palm_pixels,
        manual_palm_corner_uncertainty_px=5 if manual_palm_pixels else None,
        manual_rim_corner_uncertainty_px=3, rim_error_px=rim_error,
        rim_annotation_method='Intersections of manually read visible straight edge segments; rounded arc points are excluded.',
        manually_read_edge_segments_px=edge_segments,
        cube_rotation_camera=cube_R.tolist(), cube_center_camera_m=cube_t.tolist(),
        hypotheses=hypotheses, small_sticker_diagnostics=sticker_rows,
        side_detection_rows=side_detection_rows,
        old_wrist_bridge_used=False, marker_to_model_registration_verified=False,
        hardware_execution_allowed=False, motor_commands_sent=0,
        limitations=[
            'No calibrated camera-to-hand mount; pixel jitter is not the full pose uncertainty.',
            'Visible rounded body corners were read manually. Cube size is the user-measured nominal 55mm.',
            'Sticker corners do not define the 55mm rim pose in this fit.',
            'No servo or camera was opened. No control parameter, joint range or motor target was changed.',
        ],
    )
    (FOLDER / 'direct_reference_cube_pose.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({k:report[k] for k in ('status','detected_ids','camera_sample_dt_s','rim_error_px',
        'cube_center_camera_m','hypotheses','small_sticker_diagnostics')},indent=2))
    print('SIDE_IDS', [(f['image'],f['ids']) for f in side_detection_rows])


if __name__ == '__main__':
    main()
