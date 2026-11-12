"""Deterministic geometric inspection of the user's ruler photo.

Preserves source pixels apart from geometric resampling; no generated content.
Deskew is not 3-D clearance calibration.
"""
import hashlib
import json
from pathlib import Path
import shutil

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
SOURCE = Path(r'C:\Users\김준서\Documents\카카오톡 받은 파일\KakaoTalk_20261011_192435215.jpg')
OUT = ROOT / 'support_ruler_20261011'


def save(path, img):
    ok, data = cv2.imencode(path.suffix, img)
    if not ok:
        raise RuntimeError(f'Encoding failed: {path}')
    data.tofile(str(path))


def main():
    OUT.mkdir(exist_ok=True)
    source_bytes = SOURCE.read_bytes()
    source_hash = hashlib.sha256(source_bytes).hexdigest()
    img = cv2.imdecode(np.frombuffer(source_bytes, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise RuntimeError('Cannot decode source')
    copied = OUT / 'source.jpg'
    if copied.exists() and hashlib.sha256(copied.read_bytes()).hexdigest() != source_hash:
        raise RuntimeError('Existing source differs; do not overwrite')
    if not copied.exists():
        shutil.copyfile(SOURCE, copied)
    h, w = img.shape[:2]
    # Approximate printed numeral centres in the 1824x1368 conversation preview.
    # They constrain image roll only, not hidden motor/support gap or depth.
    labels = np.array([[16,887,839],[17,950,817],[18,1011,793],
                       [19,1073,770],[20,1135,748],[21,1197,725],[22,1259,701]], float)
    labels[:, 1:] *= np.array([w / 1824, h / 1368])
    coeff = np.polyfit(labels[:, 0], labels[:, 1:], 1)
    dx, dy = coeff[0]
    angle = float(np.degrees(np.arctan2(dy, dx)))
    # OpenCV's positive angle rotates visually counterclockwise.
    M = cv2.getRotationMatrix2D((w / 2, h / 2), angle, 1)
    corners = np.array([[0,0],[w,0],[w,h],[0,h]], float)
    warped_corners = corners @ M[:, :2].T + M[:, 2]
    minimum, maximum = warped_corners.min(0), warped_corners.max(0)
    M[:, 2] -= minimum
    size = tuple(np.ceil(maximum - minimum).astype(int))
    rotated = cv2.warpAffine(img, M, size, flags=cv2.INTER_CUBIC,
                              borderMode=cv2.BORDER_CONSTANT, borderValue=(32,32,32))
    save(OUT / 'deskewed.jpg', rotated)
    region = np.array([[500,380],[1390,380],[1390,1030],[500,1030]], float)
    region *= np.array([w / 1824, h / 1368])
    roi = region @ M[:, :2].T + M[:, 2]
    x0,y0 = np.maximum(np.floor(roi.min(0)).astype(int), 0)
    x1,y1 = np.minimum(np.ceil(roi.max(0)).astype(int), size)
    crop = rotated[y0:y1,x0:x1]
    save(OUT / 'support_detail.jpg', crop)
    preview = cv2.resize(crop, (1800, round(crop.shape[0]*1800/crop.shape[1])))
    save(OUT / 'support_detail_preview.jpg', preview)
    report = dict(source=str(SOURCE), source_sha256=source_hash,
                  source_size_px=[w,h], method='OpenCV affine rotation only; original-pixel resampling',
                  ruler_clockwise_correction_deg=-angle,
                  ruler_line_px_per_mm=float(np.hypot(dx,dy)/10),
                  landmarks='Manually estimated numeral centres; provisional local scale only',
                  affine_source_to_output=M.tolist(),
                  perspective_rectified=False, motor_to_wood_gap_mm=None,
                  caveat='Rotation removes image roll, not out-of-plane perspective. Cables obscure the closest motor/support surfaces; no exact clearance inferred.')
    (OUT / 'analysis.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
