"""Review the saved loaded run and export its raw rotation; no devices."""
import hashlib
import json
import os
from pathlib import Path
import cv2 as cv
import numpy as np
from marker_detector import MarkerDetector

ROOT = Path(__file__).resolve().parent
FOLDER = ROOT / 'cube_supported_rotation_trial_20261023_034640_750784'
PHASE = 'stage_4_supported_cube_rotation'


def read_image(path):
    return cv.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv.IMREAD_COLOR)


def main():
    blob = (FOLDER / 'result.json').read_bytes()
    r = json.loads(blob)
    candidate = json.loads((ROOT / 'right_hand_rotation_20261023/supported_loaded_candidate.json').read_bytes())
    targets = np.array(candidate['goal_ticks'])
    samples = r['samples']
    suffix = [s for s in samples if s['phase'] == PHASE]
    expected = np.vstack([suffix[0]['goal'], targets[:-1]])
    exact = bool(np.array_equal(np.array([s['goal'] for s in suffix]), expected))
    final = bool(np.array_equal(r['final_on_state']['goal'], targets[-1]))
    assert r['status'] == 'cube_supported_reorientation_path_complete'
    assert len(suffix) == len(targets) == 256 and exact and final
    assert r['all_torque_off_verified'] and r['settings_restored']
    phases = {}
    for name in dict.fromkeys(s['phase'] for s in samples):
        group = [s for s in samples if s['phase'] == name]
        phases[name] = dict(samples=len(group), start_s=group[0]['t'], end_s=group[-1]['t'],
            min_voltage_v=float(np.min([s['voltage_v'] for s in group])),
            peak_current_ma=float(np.max(abs(np.array([s['current_ma'] for s in group])))),
            max_tracking_rad=float(np.max([s['tracking_rad'] for s in group])))
    out = dict(source_result_sha256=hashlib.sha256(blob).hexdigest(), status=r['status'],
        recorded_rotation_targets=len(suffix), exact_previous_targets=exact, final_target_exact=final,
        all_torque_off_verified=r['all_torque_off_verified'], settings_restored=r['settings_restored'],
        cube_placement_confirmed=r['cube_placement_confirmed'], cube_removal_confirmed=r['cube_removal_confirmed'],
        min_voltage_v=min(x['min_voltage_v'] for x in phases.values()),
        peak_current_ma=max(x['peak_current_ma'] for x in phases.values()),
        max_tracking_rad=max(x['max_tracking_rad'] for x in phases.values()), phases=phases,
        camera=r['camera_recording'], physical_desired_rotation_verified=False,
        agent_hardware_commands_sent=0)
    frames = json.loads((FOLDER / 'camera/frames.json').read_bytes())['frames']
    selected = [f for f in frames if f['phase'] in (PHASE, 'stage_2_fixed_hold', 'supported_rotation_observation')]
    assert selected and all(f['image_signal_usable'] for f in selected)
    period = (selected[-1]['camera_read_monotonic'] - selected[0]['camera_read_monotonic']) / (len(selected) - 1)
    first = read_image(FOLDER / 'camera' / selected[0]['image'])
    os.chdir(FOLDER)
    writer = cv.VideoWriter('recorded_loaded_rotation.mp4', cv.VideoWriter_fourcc(*'mp4v'), 1 / period,
                            (first.shape[1], first.shape[0]))
    assert writer.isOpened()
    detector = MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),
                              range(6), hand_id=40, required_cube_count=1)
    marker_rows = []
    sheet = np.zeros((3 * 240, 3 * 426, 3), np.uint8)
    key_indices = np.rint(np.linspace(0, len(selected) - 1, 9)).astype(int).tolist()
    for i, f in enumerate(selected):
        image = read_image(FOLDER / 'camera' / f['image'])
        assert image is not None and image.shape == first.shape
        writer.write(image)
        corners, ids, _ = detector.detectMarkers(cv.cvtColor(image, cv.COLOR_BGR2GRAY))
        ids = [] if ids is None else list(map(int, ids.ravel()))
        points = {str(j): np.asarray(c).reshape(4, 2).tolist() for j, c in zip(ids, corners)}
        row = dict(**f, ids=ids, corners=points)
        if '5' in points:
            a = np.asarray(points['5'])
            edge = ((a[1] - a[0]) + (a[2] - a[3])) / 2
            row.update(cube_marker_center_px=a.mean(axis=0).tolist(),
                       cube_marker_image_edge_deg=float(np.degrees(np.arctan2(edge[1], edge[0]))))
        marker_rows.append(row)
        if i in key_indices:
            j = key_indices.index(i)
            tile = cv.resize(image, (426, 240))
            label = f"{f['camera_read_monotonic'] - selected[0]['camera_read_monotonic']:.1f}s {f['image']}"
            cv.putText(tile, label, (8, 22), cv.FONT_HERSHEY_SIMPLEX, .48, (0, 0, 0), 3)
            cv.putText(tile, label, (8, 22), cv.FONT_HERSHEY_SIMPLEX, .48, (255, 255, 255), 1)
            sheet[(j // 3) * 240:(j // 3 + 1) * 240, (j % 3) * 426:(j % 3 + 1) * 426] = tile
    writer.release()
    cv.imencode('.jpg', sheet)[1].tofile('rotation_contact_sheet.jpg')
    rotation = [x for x in marker_rows if x['phase'] == PHASE and '5' in x['corners']]
    marker_summary = dict(analyzed_frames=len(marker_rows), rotation_frames_with_top_marker=len(rotation),
        hand_cube_pairs=sum(40 in x['ids'] and 5 in x['ids'] for x in marker_rows),
        limitation='Image edge/center changes are 2D diagnostics, not gravity-referenced yaw or a calibrated cube pose.')
    if len(rotation) >= 6:
        early, late = rotation[:5], rotation[-5:]
        marker_summary['top_marker_edge_change_deg'] = float(np.median([x['cube_marker_image_edge_deg'] for x in late]) -
                                                            np.median([x['cube_marker_image_edge_deg'] for x in early]))
        marker_summary['top_marker_center_shift_px'] = (np.median([x['cube_marker_center_px'] for x in late], axis=0) -
                                                       np.median([x['cube_marker_center_px'] for x in early], axis=0)).tolist()
    (FOLDER / 'marker_rotation_review.json').write_text(json.dumps(dict(**marker_summary, rows=marker_rows), indent=2), encoding='utf-8')
    out.update(marker_summary=marker_summary, rotation_raw_frames=len(selected),
               rotation_video_s=len(selected) * period)
    (FOLDER / 'completion_review.json').write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({k:v for k,v in out.items() if k not in ('phases','camera')}, indent=2))
    print('PHASE_METRICS', json.dumps(phases[PHASE]))


if __name__ == '__main__':
    main()
