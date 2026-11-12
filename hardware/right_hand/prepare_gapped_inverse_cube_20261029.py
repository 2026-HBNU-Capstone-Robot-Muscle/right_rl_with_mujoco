"""Build a gently closing inverse hypothesis from a screened placement gap."""
import hashlib,json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261029/gapped_inverse_cube'
OUT.mkdir(exist_ok=True)
source=ROOT/'right_hand_rotation_20261029/prepositioned_reverse_cube/candidate.npz'
old=np.load(source)
gap_review=ROOT/'right_hand_rotation_20261029/inverse_placement_gap/review.json'
rows=json.loads(gap_review.read_bytes())['passed_candidates']
print('minimum insertion depth among candidates',min(r['insertion_max_cube_depth_mm'] for r in rows))
pool=[r for r in rows if r['insertion_max_cube_depth_mm']<=.5]
if not pool:pool=rows
chosen=min(pool,key=lambda r:(r['change_l1_ticks'],r['seed_height_offset_mm']))
opened=np.asarray(chosen['placement_goal'],int);ready=old['start'].astype(int)
def interp(a,b):
    n=max(1,int(np.ceil(max(abs(b-a))/2)))
    return np.rint([a+(b-a)*i/n for i in range(1,n+1)]).astype(int)
grasp=interp(opened,ready)
keep=old['phases']!='stage_5_loaded_grasp'
goals=np.vstack([grasp,old['goals'][keep]])
phases=np.asarray(['stage_5_loaded_grasp']*len(grasp)+list(old['phases'][keep]))
path=OUT/'candidate.npz'
np.savez_compressed(path,start=opened,goals=goals,phases=phases,
    empty_preposition=old['empty_preposition'],empty_gap_open=interp(ready,opened))
seed=dict(cube_qpos=chosen['cube_qpos'],physically_registered=False,
    source='forward model held cube with bounded initial height offset',
    seed_height_offset_mm=chosen['seed_height_offset_mm'])
(OUT/'cube_seed.json').write_text(json.dumps(seed,indent=2)+'\n',encoding='utf-8')
manifest=dict(selected_gap=chosen,placement_goal=opened.tolist(),ready_goal=ready.tolist(),
    candidate_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
    source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    gap_review_sha256=hashlib.sha256(gap_review.read_bytes()).hexdigest(),
    loaded_grasp_goals=len(grasp),loaded_goals=len(goals),
    degrees_changed_for_placement={str(i+1):float((opened[i]-ready[i])*360/4096)
        for i in range(16) if opened[i]!=ready[i]},
    devices_opened=False,hardware_execution_allowed=False)
(OUT/'request.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
print(json.dumps(manifest))
