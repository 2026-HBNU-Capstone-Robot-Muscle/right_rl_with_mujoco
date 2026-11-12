"""Rough image-ratio estimate using the user-reported 20mm book thickness.

Manual image landmarks, not calibrated 3D metrology. The two fixed-body front
references are explicitly distinguished rather than silently interchanged.
"""
import hashlib
import json
from pathlib import Path
import cv2 as cv
import numpy as np

ROOT=Path(__file__).resolve().parent
PHOTO=Path('C:/Users/김준서/Documents/카카오톡 받은 파일/KakaoTalk_20261024_055554013.jpg')
OUT=ROOT/'right_hand_rotation_20261024/book20_current_review'
blob=PHOTO.read_bytes();im=cv.imdecode(np.frombuffer(blob,np.uint8),cv.IMREAD_COLOR)
if im is None:raise RuntimeError('Image decoding failed.')
height,width=im.shape[:2]
# Points visually selected on the 1824x1368 display, then scaled to the source.
# The partly cable-obscured root-motor front is intentionally approximate.
scale=np.array([width/1824,height/1368])
points={
    'root_motor_front_at_book_top':np.array([738.,740.])*scale,
    'book_front_top':np.array([850.,741.])*scale,
    'book_front_bottom':np.array([854.,958.])*scale,
    'lettered_case_front_at_book_top':np.array([1074.,737.])*scale,
}
thickness_pixels=float(np.linalg.norm(points['book_front_bottom']-points['book_front_top']))
mm_per_pixel=20./thickness_pixels
root_to_book=float(points['book_front_top'][0]-points['root_motor_front_at_book_top'][0])*mm_per_pixel
case_to_book=float(points['lettered_case_front_at_book_top'][0]-points['book_front_top'][0])*mm_per_pixel
record=dict(status='rough_photo_ratio_estimate_not_measured_3D_geometry',source_photo=str(PHOTO),
    source_sha256=hashlib.sha256(blob).hexdigest(),source_size_px=[width,height],
    book_thickness_mm_user_reported=20.,manual_landmarks_px={k:v.tolist() for k,v in points.items()},
    book_thickness_pixels=thickness_pixels,assumed_image_mm_per_pixel=mm_per_pixel,
    root_motor_front_to_book_front_estimate_mm=round(root_to_book,2),
    book_front_direction_relative_to_root_front='behind_toward_wrist_image_right',
    book_front_projection_from_fixed_motor_mount_front_mm_estimate=-10.,
    book_front_projection_for_diagnostic_sensitivity_mm=[-15.,-10.,-5.],
    lettered_case_front_to_book_front_estimate_mm=round(case_to_book,2),
    book_front_direction_relative_to_lettered_case='ahead_toward_fingers_image_left',
    geometry_calibration_verified=False,physical_measurement=False,hardware_execution_allowed=False,
    limitations=['Perspective can give different scale along the fore-aft and thickness directions.',
        'The root housing front is partly obscured by cables and differs from the large lettered case front.',
        'The +/-5mm diagnostic span is an analysis choice, not a measured or statistical error bound.',
        'This estimate is sufficient for further offline diagnostics, not an execution certificate.'])
(OUT/'photo_front_estimate.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({k:record[k] for k in ('status','source_size_px','book_thickness_pixels',
    'root_motor_front_to_book_front_estimate_mm','lettered_case_front_to_book_front_estimate_mm',
    'hardware_execution_allowed')},indent=2))
