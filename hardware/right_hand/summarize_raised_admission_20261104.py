import json
from pathlib import Path
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261104/raised_cradle_160_yaw0';cases=[]
for f in (-15,-10,-5):cases.extend(json.loads((O/f'material_front_{f}/review.json').read_bytes())['cases'])
fields=('book_front_mm','gain_scale','friction','stop','last_sample','initial_placement_passed','initial_placement_probe','max_self_mm','max_support_mm','max_cube_mm','max_tracking_rad','unapproved_range_excess_rad','external_cube_contact','final_cube_supported_by_hand','longest_no_hand_contact_seconds')
report=dict(total=len(cases),passed=sum(c['passed'] for c in cases),failed=[{k:c[k] for k in fields} for c in cases if not c['passed']],hand_passed=sum(c['passed'] for c in json.loads((O/'loaded_hand_dynamics/review.json').read_bytes())['cases']),empty_all_passed=json.loads((O/'empty_recovery_screen/review.json').read_bytes())['all_passed'],nominal_model_90_verified=False,motion_admission=False)
(O/'admission_summary.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report))
