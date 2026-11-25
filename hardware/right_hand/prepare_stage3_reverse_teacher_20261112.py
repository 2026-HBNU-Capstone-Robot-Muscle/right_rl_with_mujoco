"""Explicit opposite-direction next quarter; no positive-quarter claim."""
from pathlib import Path
import hashlib
import json
ROOT=Path(__file__).resolve().parent
source=ROOT/'stage3_contact_avoidance_teacher_20261112.py'
target=ROOT/'stage3_reverse_teacher_20261112.py'
if target.exists(): raise RuntimeError('Preserve generated comparison')
text=source.read_text(encoding='utf-8')
before=".geom_xmat[cube].reshape(3,3),initial['up'],90.)"
after=".geom_xmat[cube].reshape(3,3),initial['up'],-90.)"
if text.count(before)!=1: raise RuntimeError('Unexpected target source')
text=text.replace(before,after,1).replace('relative_yaw_target_deg=args.increment','relative_yaw_target_deg=-args.increment',1)
compile(text,str(target),'exec'); target.write_text(text,encoding='utf-8')
(ROOT/'right_hand_rotation_20261112/stage3_preparation/reverse_teacher_provenance.json').write_text(json.dumps({
    'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'new_sha256':hashlib.sha256(target.read_bytes()).hexdigest(),
    'requested_next_direction_deg':-90,'two_positive_quarters_claimed':False,'native_guards_changed':False},indent=2)+'\n',encoding='utf-8')
