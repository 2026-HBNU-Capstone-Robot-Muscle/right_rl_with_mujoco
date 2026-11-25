"""Build the user-taught middle-clear, thumb-push, thumb-wrist transfer offline."""
from pathlib import Path
import hashlib,json
import numpy as np
import leap_measured_grip_hold_20261117 as pose
from measured_grip_small_cycle_plan_20261117 import validate_rows

ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261118/quarter_turn_fixed_grip'
OUT=BASE/'taught_palm_transfer'
FIRST=ROOT/'right_hand_rotation_20261104/forward_hold/candidate.npz'

def interpolate(start,target):
    n=max(1,int(np.ceil(np.max(abs(target-start))/2)))
    return np.rint(start+(target-start)*np.arange(1,n+1)[:,None]/n).astype(int)

def main():
    first=np.load(FIRST,allow_pickle=False);_,high,low=pose.bounds()
    stages=('middle_clear','cube_pushed','thumb_wrist_clear')
    refs={name:json.loads((BASE/(name+'_reference.json')).read_bytes()) for name in stages}
    for name,r in refs.items():
        if hashlib.sha256((ROOT/r['source']).read_bytes()).hexdigest()!=r['source_sha256']:
            raise RuntimeError('Recorded source changed: '+name)
    end=first['goals'][-1].astype(int);grip=pose.reference()[2]
    middle=end.copy();middle[8:12]=refs['middle_clear']['actual_ticks'][8:12]
    # Keep root motion inward of the original native target limit. Do not
    # import the other fingers' observed rest angles into the loaded motion.
    adjustments=[]
    for i in np.flatnonzero((middle<low)|(middle>high)):
        before=int(middle[i]);middle[i]=int(np.clip(middle[i],np.ceil(low[i])+4,high[i]))
        adjustments.append(dict(id=int(i+1),measured=before,proposed=int(middle[i])))
    ready=middle.copy();ready[:4]=refs['middle_clear']['actual_ticks'][:4]
    pushed=ready.copy();pushed[:4]=refs['cube_pushed']['actual_ticks'][:4]
    wrist=pushed.copy();wrist[:4]=refs['thumb_wrist_clear']['actual_ticks'][:4]
    waypoints=[('transfer_middle_clear',middle),('transfer_thumb_prepare',ready),
               ('transfer_thumb_push',pushed),('transfer_thumb_wrist_clear',wrist),
               ('fixed_grip_close',grip)]
    rows=list(first['goals'].astype(int));phases=list(first['phases'].astype(str));previous=end.copy()
    phase_ends={};targets={}
    for phase,target in waypoints:
        part=interpolate(previous,target);rows.extend(part);phases.extend([phase]*len(part))
        phase_ends[phase]=len(rows)-1;targets[phase]=target.tolist();previous=target.copy()
    rows.extend([grip.copy()]*134);phases.extend(['fixed_grip_hold']*134)
    goals=np.asarray(rows,dtype=int);validate_rows(first['start'],goals)
    if not np.array_equal(goals[:1417],first['goals']) or not np.array_equal(goals[-1],grip):
        raise RuntimeError('Fixed rotation/grip contract changed')
    OUT.mkdir(exist_ok=True)
    np.savez_compressed(OUT/'candidate.npz',start=first['start'],goals=goals,phases=np.array(phases,dtype='<U64'))
    report=dict(kind='role_sequenced_user_taught_palm_transfer',total_goals=len(goals),
        first1417_exactly_preserved=True,fixed_grip_exactly_preserved=True,phase_end_indices=phase_ends,
        stage_target_ticks=targets,inward_native_limit_adjustments=adjustments,new_joint_limits_added=False,
        movement_period_s=.075,maximum_goal_step_ticks=2,
        unchanged_index_ring_until_final_close=True,unchanged_middle_during_thumb_stages=True,
        unchanged_thumb_during_middle_clear=True,cube_transfer_physically_verified=False,
        whole_reference_hand_poses_replayed=False,candidate_sha256=hashlib.sha256((OUT/'candidate.npz').read_bytes()).hexdigest(),
        source_references={name:dict(path=str((BASE/(name+'_reference.json')).relative_to(ROOT)),
            sha256=hashlib.sha256((BASE/(name+'_reference.json')).read_bytes()).hexdigest()) for name in stages})
    (OUT/'construction_review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False))

if __name__=='__main__':main()
