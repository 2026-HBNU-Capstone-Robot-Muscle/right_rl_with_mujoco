import hashlib,json,shutil
from pathlib import Path
root=Path(__file__).resolve().parent
out=root/'right_hand_rotation_20261024/loaded_quarter_trial_20261025/archived_v3_runtime';out.mkdir(exist_ok=True)
names=('loaded_quarter_trial_plan_20261025.py','loaded_quarter_trial_gate_20261025.py','screen_loaded_quarter_trial_20261025.py','leap_loaded_quarter_trial_20261025.py','build_loaded_quarter_trial_runtime_20261025.py','test_loaded_quarter_trial_20261025.py')
sha={}
for name in names:
    target=out/name
    if target.exists():raise RuntimeError('Archive already exists; do not overwrite')
    shutil.copyfile(root/name,target);sha[name]=hashlib.sha256(target.read_bytes()).hexdigest()
(out/'sha256.json').write_text(json.dumps(sha,indent=2),encoding='utf-8')
print('V3_ARCHIVED',len(sha))
