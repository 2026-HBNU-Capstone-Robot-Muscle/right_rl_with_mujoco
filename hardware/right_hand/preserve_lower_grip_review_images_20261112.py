"""Preserve four already-reviewed original frames as selected private evidence."""
import hashlib
import json
import shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parent
folder=ROOT/'right_hand_rotation_20261112/lower_grip_actual_review'
path=folder/'frame_selection.json';rows=json.loads(path.read_bytes())
for row,name in zip(rows,('actual_receiving.jpg','actual_thumb_push.jpg','actual_support_transfer.jpg','actual_ring_stop.jpg')):
    source=ROOT/'lower_grip_trial_20261112_162806_610535/camera'/row['frame']['image'];target=folder/name
    shutil.copyfile(source,target);row['copied_review_image']=name;row['sha256']=hashlib.sha256(target.read_bytes()).hexdigest()
path.write_text(json.dumps(rows,indent=2,ensure_ascii=False),encoding='utf-8')
print('Four selected original review images preserved; raw recording directory excluded.')
