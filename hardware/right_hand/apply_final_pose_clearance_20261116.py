"""Apply only the separately diagnosed 4tick EMPTY thumb clearance."""
import json
import shutil
import numpy as np
from final_grip_pose_plan_20261116 import OUT,CANDIDATE,MANIFEST,MOVE,HOLD,sha
from calibrated_empty_ring_margin_20261028 import interpolate

selected=json.loads((OUT/'clearance_search.json').read_bytes())['selected']
if selected['motor'] != 3 or selected['delta'] != 4 or selected['self_mm'] != 0:
    raise RuntimeError('Expected separately inspected correction changed')
backup=OUT/'straight_1tick_attempt';backup.mkdir(exist_ok=False)
for p in (CANDIDATE,MANIFEST):shutil.copy2(p,backup/p.name)
z=dict(np.load(CANDIDATE,allow_pickle=False));a=z['start'];b=z['goals'][-1].copy();b[2]+=4
# Two ticks is the existing bound; slowing did not fix final gravity deflection.
goals=interpolate(a,b);phases=[MOVE]*len(goals)+[HOLD]*40
np.savez_compressed(CANDIDATE,start=a,goals=np.vstack([goals,np.tile(b,(40,1))]),phases=np.asarray(phases))
offset=[0]*16;offset[2]=4
m=json.loads(MANIFEST.read_bytes());m.update(candidate_sha256=sha(CANDIDATE),empty_pose_max_step_ticks=2,
    empty_clearance_offset_ticks=offset,physical_target_ticks=b.tolist(),
    correction_reason='EMPTY no-cube gravity deflection: TH distal/MF base clearance; no limits or thresholds changed',
    correction_angle_degrees=4*360/4096)
MANIFEST.write_text(json.dumps(m,indent=2)+'\n',encoding='utf-8')
print('ID3 +4tick /0.3515625deg EMPTY clearance; original pose preserved in manifest/reference')
