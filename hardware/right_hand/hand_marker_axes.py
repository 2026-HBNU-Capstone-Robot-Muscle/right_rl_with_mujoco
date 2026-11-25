"""Read marker orientation from the supplied photograph. No camera or motor access."""
import json
from pathlib import Path
import cv2
import numpy as np

root = Path(__file__).resolve().parent
measurement = json.loads((root / 'hand_registration_measurements.json').read_text(encoding='utf-8'))
photo = Path(measurement['photo'])
frame = cv2.imdecode(np.frombuffer(photo.read_bytes(), dtype=np.uint8), cv2.IMREAD_COLOR)
if frame is None:
    raise SystemExit('Could not decode supplied photograph')
detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
                                  cv2.aruco.DetectorParameters())
corners, ids, _ = detector.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
matches = [] if ids is None else [c.reshape(4, 2) for c, i in zip(corners, ids.ravel()) if int(i) == 40]
if len(matches) != 1:
    raise SystemExit(f'Expected one ID40, got {len(matches)}; no calibration applied.')
c = matches[0]
x = (c[1] + c[2] - c[0] - c[3]) / 2
y = (c[0] + c[1] - c[2] - c[3]) / 2
report = {'status': 'photo_axes_only_not_calibration', 'photo': str(photo),
          'photo_size': [frame.shape[1], frame.shape[0]],
          'id40_canonical_corners_px': c.tolist(),
          'marker_positive_x_image_direction': (x / np.linalg.norm(x)).tolist(),
          'marker_positive_y_image_direction': (y / np.linalg.norm(y)).tolist(),
          'image_direction_convention': 'x right, y down; vectors are not 3D hand coordinates',
          'motors_accessed': False, 'camera_opened': False}
output = root / 'hand_marker_axes_result.json'
output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps(report, ensure_ascii=False, indent=2))
print(f'Saved: {output}')
