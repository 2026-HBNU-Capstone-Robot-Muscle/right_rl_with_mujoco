"""Record the specifically inspected EMPTY images without continuous claims."""
import hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
F=ROOT/'supported_inverse_empty_20261029_225256_285948';O=F/'offline_review'
review=json.loads((O/'review.json').read_bytes())
names=['frame_00850.jpg','frame_01085.jpg','frame_01522.jpg']
report=dict(source_review_sha256=hashlib.sha256((O/'review.json').read_bytes()).hexdigest(),
    inspected_images=[dict(path=(F/'camera'/name).relative_to(ROOT).as_posix(),sha256=hashlib.sha256((F/'camera'/name).read_bytes()).hexdigest()) for name in names],
    empty_hand_seen_in_selected_images=True,no_person_hand_in_selected_images=True,
    obvious_finger_or_support_interference_not_seen_in_selected_views=True,
    continuous_contact_clearance_not_certified=True,physical_cube_rotation_not_tested=True,
    note='Prepared gap, turn ending and final OFF image inspected; a Cube is absent. Isolated views do not certify hidden surfaces or continuous clearance.')
(O/'visual_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps(report))
