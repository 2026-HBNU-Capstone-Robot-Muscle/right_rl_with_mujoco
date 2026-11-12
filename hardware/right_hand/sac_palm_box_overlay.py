"""Project provisional MuJoCo palm collision boxes onto the real support photo."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from hand_registration_geometry import candidate
from sac_registration_from_anchor import detect, marker_pose

ROOT = Path(__file__).resolve().parent
FOLDER = ROOT / 'sequence_vision_20261007_000611_625058'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--empty-image', type=Path, default=FOLDER/'preflight_not_ready.png')
    parser.add_argument('--placed-image', type=Path, default=FOLDER/'placement_at_space.png')
    parser.add_argument('--output-stem', default='sac_palm_box_overlay')
    parser.add_argument('--model-offset-world-mm', nargs=3, type=float,
                        metavar=('X', 'Y', 'Z'), default=(0., 0., 0.))
    args = parser.parse_args()
    with np.load(ROOT / 'camera1_720.npz') as camera:
        K, dist = camera['K'].copy(), camera['dist'].copy()
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    _, before_corners, before_ids, _ = detect(args.empty_image, dictionary)
    frame = cv2.imdecode(np.frombuffer(args.placed_image.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError(f'Cannot read {args.placed_image}')
    after_corners, found, _ = cv2.aruco.ArucoDetector(
        dictionary, cv2.aruco.DetectorParameters()).detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    after_ids = [] if found is None else [int(i) for i in found.ravel()]
    R_h0, t_h0, _ = marker_pose(before_corners, before_ids, 40, .0282, K, dist)
    R_a0, t_a0, _ = marker_pose(before_corners, before_ids, 41, .030, K, dist)
    R_a1, t_a1, _ = marker_pose(after_corners, after_ids, 41, .030, K, dist)
    R_h1 = R_a1 @ R_a0.T @ R_h0
    t_h1 = t_a1 + R_a1 @ R_a0.T @ (t_h0-t_a0)
    hand = candidate()
    R_wm = np.asarray(hand['hand_to_world_candidate'])
    g_m = np.asarray(hand['grasp_site_in_marker_m_candidate'])
    marker_world = np.array([.11, 0, .03]) - R_wm @ g_m
    boxes = json.loads((ROOT/'sac_palm_box_corners_result.json').read_text(encoding='utf-8'))
    colors = {'palm_collision_6': (255,255,0),
              'palm_collision_7': (255,0,255),
              'palm_collision_8': (0,255,255)}
    projected_report = {}
    for name, box in boxes.items():
        world = np.asarray(box['corners_world_m']) + np.asarray(args.model_offset_world_mm) / 1000
        marker = (R_wm.T @ (world-marker_world).T).T
        camera_xyz = (R_h1 @ marker.T).T+t_h1
        pixels, _ = cv2.projectPoints(camera_xyz, np.zeros(3), np.zeros(3), K, dist)
        pixels = pixels.reshape(-1,2)
        color = colors[name]
        for i in range(8):
            for mask in (1,2,4):
                j = i ^ mask
                if i < j:
                    cv2.line(frame, tuple(np.rint(pixels[i]).astype(int)),
                             tuple(np.rint(pixels[j]).astype(int)), color, 2)
        mean = np.mean(pixels, axis=0)
        cv2.putText(frame, name[-1], tuple(np.rint(mean).astype(int)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        projected_report[name] = {'center_px': mean.tolist(),
                                  'pixel_bbox_xyxy': [float(pixels[:,0].min()),float(pixels[:,1].min()),
                                                      float(pixels[:,0].max()),float(pixels[:,1].max())]}
    output = ROOT/f'{args.output_stem}.png'
    output.write_bytes(cv2.imencode('.png',frame)[1].tobytes())
    report = {'source': str(args.placed_image),
              'model_offset_world_mm': list(args.model_offset_world_mm),
              'projected_boxes': projected_report,
              'calibration': 'provisional ID40/ID41 to placed ID41 bridge',
              'registration_verified': False, 'hardware_authorized': False}
    (ROOT/f'{args.output_stem}_result.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
