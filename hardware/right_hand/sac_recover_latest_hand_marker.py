"""Try to decode the hand reference in a saved image; no devices opened."""
import json
import argparse
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
parser = argparse.ArgumentParser()
parser.add_argument('--folder', default='sequence_vision_20261008_184238_359328')
parser.add_argument('--output-file', default='sac_recover_latest_hand_marker_result.json')
parser.add_argument('--images', nargs='+', default=[
    'preflight_not_ready', 'stage_0_not_ready', 'placement_at_space'])
args = parser.parse_args()
FOLDER = ROOT / args.folder
IMAGES = [FOLDER / f'{name}.png' for name in
          args.images]
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
settings = cv2.aruco.DetectorParameters()
settings.minMarkerPerimeterRate = .008
detector = cv2.aruco.ArucoDetector(dictionary, settings)
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
rows = []
for path in IMAGES:
    frame = cv2.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError(f'Cannot read {path}')
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    variants = {
        'raw': gray,
        'clahe': clahe.apply(gray),
        'equalized': cv2.equalizeHist(gray),
        'gamma_070': np.uint8(255 * (gray / 255.0) ** .7),
        'gamma_130': np.uint8(255 * (gray / 255.0) ** 1.3),
        'blur_sharpen': cv2.addWeighted(gray, 2.0, cv2.GaussianBlur(gray, (0, 0), 1.5), -1.0, 0),
    }
    for name, img in variants.items():
        for scale in (1.0, 1.5, 2.0):
            work = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC) if scale != 1 else img
            corners, ids, _ = detector.detectMarkers(work)
            if ids is None:
                continue
            for corner, marker_id in zip(corners, ids.ravel()):
                rows.append({'image': path.name, 'variant': name, 'scale': scale, 'id': int(marker_id),
                             'center_px': (corner.reshape(4, 2).mean(axis=0) / scale).tolist(),
                             'corners_px': (corner.reshape(4, 2) / scale).tolist()})
out = {'sources': [str(p) for p in IMAGES], 'detections': rows,
       'by_image': {p.name: sorted(set(r['id'] for r in rows if r['image'] == p.name)) for p in IMAGES},
       'hand_reference_found': any(row['id'] in (40, 41) for row in rows),
       'hardware_authorized': False}
(ROOT / args.output_file).write_text(
    json.dumps(out, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print(json.dumps({'by_image': out['by_image'], 'hand_reference_found': out['hand_reference_found']}, indent=2))
