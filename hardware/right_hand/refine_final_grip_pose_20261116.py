"""Slow only the new empty pose segment; retain the failed first screen."""
import json
import shutil
import numpy as np
from final_grip_pose_plan_20261116 import OUT,CANDIDATE,MANIFEST,MOVE,HOLD,sha

backup=OUT/'straight_2tick_attempt'
backup.mkdir(exist_ok=False)
for p in (CANDIDATE,MANIFEST):
    shutil.copy2(p,backup/p.name)
z=dict(np.load(CANDIDATE,allow_pickle=False));a=z['start'];b=z['goals'][-1]
count=int(max(abs(b-a)))
goals=np.rint([a+(b-a)*i/count for i in range(1,count+1)]).astype(int)
phases=[MOVE]*len(goals)+[HOLD]*40
np.savez_compressed(CANDIDATE,start=a,goals=np.vstack([goals,np.tile(b,(40,1))]),phases=np.asarray(phases))
m=json.loads(MANIFEST.read_bytes());m.update(candidate_sha256=sha(CANDIDATE),empty_pose_max_step_ticks=1)
MANIFEST.write_text(json.dumps(m,indent=2)+'\n',encoding='utf-8')
print('Only new empty segment slowed: maximum 1tick/75ms;',len(goals),'move goals')
