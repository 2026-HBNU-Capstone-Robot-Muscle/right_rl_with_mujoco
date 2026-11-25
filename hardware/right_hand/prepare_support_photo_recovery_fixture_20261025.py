import json
from pathlib import Path
from loaded_quarter_trial_plan_20261025 import ROOT,LoadedQuarterTrialPlan
source=ROOT/'quarter_support_photo_20261025_214901_644144/fresh_off_request.json'
out=ROOT/'right_hand_rotation_20261024/rigid_contact_review_20261025/off_pose_214901';out.mkdir(exist_ok=True)
(out/'request.json').write_bytes(source.read_bytes())
p=LoadedQuarterTrialPlan(out/'request.json');p.plan(p.source_position())
print('OFF REQUEST ACCEPTED',out/'request.json')
