"""Reuse the EMPTY-tested tape, placing the cube after reverse pre-grasp.

This generates offline hypotheses only. No hardware protocol is changed.
"""
import hashlib,json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261029/prepositioned_reverse_cube'
OUT.mkdir(exist_ok=True)
source=ROOT/'right_hand_rotation_20261029/reverse_yaw_empty/reverse_cycle.npz'
original=np.load(source)
goals=original['goals'].astype(int)
phases=original['phases'].astype(str)
labels=list(dict.fromkeys(phases))
print('PHASES',[(p,int(np.sum(phases==p))) for p in labels])
grasp=goals[phases=='stage_5_loaded_grasp']
assert len(grasp)>0
ready=grasp[-1].copy()
# The first goal repeats the already reached placement pose; it is not a new grasp.
keep=phases!='stage_5_loaded_grasp'
new_goals=np.vstack([ready[None,:],goals[keep]])
new_phases=np.asarray(['stage_5_loaded_grasp',*phases[keep]])
path=OUT/'candidate.npz'
np.savez_compressed(path,start=ready,goals=new_goals,phases=new_phases,
    empty_preposition=grasp)
endpoint=ROOT/'right_hand_rotation_20261029/reverse_cube_scout/forward_endpoint/review.json'
review=json.loads(endpoint.read_bytes())
print('ENDPOINT CASE KEYS',list(review['cases'][0]))
row=review['cases'][0]
key=next(k for k in row if 'held' in k and 'qpos' in k)
cube=row[key]
seed=dict(cube_qpos=cube,source=endpoint.relative_to(ROOT).as_posix(),
    source_sha256=hashlib.sha256(endpoint.read_bytes()).hexdigest(),source_field=key,
    physically_registered=False)
(OUT/'held_cube_seed.json').write_text(json.dumps(seed,indent=2)+'\n',encoding='utf-8')
manifest=dict(candidate=path.relative_to(ROOT).as_posix(),
    candidate_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
    original_empty_tested_tape=source.relative_to(ROOT).as_posix(),
    original_empty_tested_tape_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    place_after_empty_preposition=True,preposition_goals=len(grasp),
    loaded_goals=len(new_goals),placement_goal=ready.tolist(),
    all_loaded_goal_vectors_already_in_tested_empty_cycle=all(
        tuple(q) in {tuple(x) for x in goals} for q in new_goals),
    devices_opened=False,hardware_execution_allowed=False)
(OUT/'request.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
print(json.dumps(manifest))
