"""Compose only previously executed forward/reverse goal vectors."""
import json,hashlib
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261030/yaw_roundtrip'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    OUT.mkdir(parents=True,exist_ok=True)
    fpath=ROOT/'right_hand_rotation_20261028/observed90_prefix638/full_loaded_27/loaded_path.npz'
    ipath=ROOT/'right_hand_rotation_20261029/gapped_inverse_cube/candidate.npz';f=np.load(fpath);i=np.load(ipath)
    u,n=np.unique(f['phases'],return_counts=True)
    print('FORWARD',dict(keys=f.files,phases={str(k):int(v) for k,v in zip(u,n)}),flush=True)
    release_start=int(np.flatnonzero(f['phases']=='release')[0]);prefix=f['goals'][:release_start]
    reverse=f['goals'][f['phases']=='yaw'][::-1];release=i['goals'][i['phases']=='stage_9_loaded_open']
    assert len(reverse)==638 and np.array_equal(prefix[-1],reverse[0])
    assert np.array_equal(i['goals'][i['phases']=='stage_7_horizontal_yaw'],reverse)
    assert np.max(abs(reverse[-1]-release[0]))<=2
    goals=np.vstack([prefix,reverse,np.repeat(reverse[-1][None,:],40,0),release,np.repeat(f['start'][None,:],20,0)])
    mapping={'grasp':'stage_5_loaded_grasp','grasp_hold':'stage_6_grasp_hold','yaw':'stage_7_horizontal_yaw','yaw_hold':'stage_8_horizontal_hold'}
    phases=np.r_[[mapping[str(p)] for p in f['phases'][:release_start]],['stage_7_horizontal_yaw']*638,['stage_8_horizontal_hold']*40,['stage_9_loaded_open']*len(release),['stage_10_remove_hold']*20]
    all_old={tuple(q) for q in np.vstack([f['start'],f['goals'],i['goals']])};assert all(tuple(q) in all_old for q in goals)
    assert np.max(abs(np.diff(np.vstack([f['start'],goals]),axis=0)))<=2
    p=OUT/'candidate.npz';np.savez_compressed(p,start=f['start'],goals=goals,phases=phases,forward_hold_goal=prefix[-1],inverse_hold_goal=reverse[-1],forward_end_sample=len(prefix))
    report=dict(status='forward_then_inverse_without_cube_replacement_candidate',candidate_sha256=sha(p),sources=[dict(path=q.relative_to(ROOT).as_posix(),sha256=sha(q)) for q in (fpath,ipath)],
        loaded_goals=len(goals),nominal_loaded_seconds=len(goals)*.075,forward_yaw_goals=638,inverse_yaw_goals=638,new_joint_goal_vectors=0,
        entire_composed_cycle_not_yet_physically_tested=True,physics_review_pending=True,hardware_execution_allowed=False,devices_opened=False)
    (OUT/'proposal.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report))
    for path in (ROOT/'right_hand_rotation_20261028/observed90_prefix638/full_loaded_27/review.json',ROOT/'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json'):
        r=json.loads(path.read_bytes());print('REVIEW_KEYS',str(path.relative_to(ROOT)),list(r.keys()))
        cases=r.get('cases',r.get('loaded_physics',[]));print('CASE_KEYS',list(cases[0]) if cases else [])
        for c in cases:
            if c.get('book_front_mm')==-10 and c.get('gain_scale')==1 and c.get('friction')==.3:
                print('NOMINAL_SEED_FIELDS',json.dumps({k:v for k,v in c.items() if any(s in k for s in ('cube','qpos','start','settled','initial','yaw','turn'))}));break
if __name__=='__main__':main()
