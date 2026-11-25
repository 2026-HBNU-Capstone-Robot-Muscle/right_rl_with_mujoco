"""Replay a successful new grip path and append a longer identical-goal hold."""
from pathlib import Path
import argparse,hashlib,json,sys,time
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
import numpy as np,mujoco
from canonical_grip_dynamics_20261105 import run
from research_rotation_body_hold_20261104 import native


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--parent',default='level_normal1p2');a=ap.parse_args()
    base=ROOT/'right_hand_rotation_20261117/opposed_lower_grip'/a.parent
    previous=json.loads((base/'review.json').read_bytes())
    if not previous.get('common_grip_passed'):raise RuntimeError('Source grip has not passed')
    out=base/'extended_hold_30s';out.mkdir(exist_ok=False)
    source=ROOT/'right_hand_rotation_20261116/opposed_lower_grip/index_base_support/palm_offset/independent_initial.npz'
    model=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
    if sha(source)!=previous['source_sha256'] or sha(model)!=previous['model_sha256']:
        raise RuntimeError('Parent source/model changed')
    z=dict(np.load(source,allow_pickle=False));old=np.load(base/'research_goals.npz',allow_pickle=False)
    rows=np.vstack([old['goals'],np.tile(old['goals'][-1],(400,1))])
    if not np.array_equal(old['goals'][-134:],np.tile(old['goals'][-1],(134,1))):raise RuntimeError('Source hold not constant')
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':model.read_bytes()});lo,hi,_,_=native(m)
    if np.any(rows<lo) or np.any(rows>hi) or np.max(abs(np.diff(np.vstack([old['start'],rows]),axis=0)))>2:
        raise RuntimeError('Goal contract failed')
    began=time.perf_counter();print('Replaying whole path, then30 extra seconds with identical goals.',flush=True)
    result=run(m,z,rows,1,out/'end_state.npz',out/'trajectory.npz',True,5,('if_tip','th_tip'))
    np.savez_compressed(out/'candidate.npz',start=old['start'],goals=rows)
    report=dict(status='extended_supported_opposed_grip_hold',source_sha256=sha(source),model_sha256=sha(model),
        parent_review_sha256=sha(base/'review.json'),candidate_sha256=sha(out/'candidate.npz'),
        appended_hold_seconds=30.,total_goal_seconds=len(rows)*.075,result=result,
        source_path_replayed_from_beginning=True,cube_reset_after_start=False,devices_opened=False,
        hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(passed=result['whole_grip_task_passed'],tilt=result['final_up_tilt_deg'],
        tail=result['settled_tail'],stop=result['stop'],report=str(out/'review.json'))),flush=True)


if __name__=='__main__':main()
