"""Keep demonstrated rotation/transfer; overlap thumb retreat with index/middle tips."""
from pathlib import Path
import hashlib,json
import numpy as np
import leap_measured_grip_hold_20261117 as pose
from build_taught_palm_transfer_20261118 import interpolate
from measured_grip_small_cycle_plan_20261117 import validate_rows

ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261118/quarter_turn_fixed_grip'
OUT=BASE/'tip_first_lower_seat'

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
    prior=np.load(BASE/'taught_palm_transfer/candidate.npz',allow_pickle=False)
    refpath=BASE/'index_middle_lower_seat_reference.json';ref=json.loads(refpath.read_bytes())
    if sha(ROOT/ref['source'])!=ref['source_sha256']:raise RuntimeError('Fourth raw reference changed')
    wristref=json.loads((BASE/'thumb_wrist_clear_reference.json').read_bytes())
    grip=pose.reference()[2];_,high,low=pose.bounds()
    prefix_count=int(np.flatnonzero(prior['phases']=='transfer_thumb_push')[-1])+1
    pushed=prior['goals'][prefix_count-1].astype(int)
    distal=pushed.copy();distal[:4]=wristref['actual_ticks'][:4]
    distal[[4,8]]=np.array(ref['actual_ticks'])[[4,8]]
    seated=distal.copy();seated[4:12]=ref['actual_ticks'][4:12]
    adjustments=[]
    for i in np.flatnonzero((seated<low)|(seated>high)):
        if i not in (7,11):raise RuntimeError('Unexpected recorded joint range')
        old=int(seated[i]);seated[i]=int(np.clip(seated[i],np.ceil(low[i])+4,high[i]))
        adjustments.append(dict(id=int(i+1),measured=old,proposed=int(seated[i])))
    raised=seated.copy();raised[[7,11]]=grip[[7,11]]
    # Start the lower push after a partial proximal curl. Fully curling while
    # the bases remain flat trapped the cube in the bounded model comparison.
    half=distal.copy();half[4:12]=np.rint((distal[4:12]+seated[4:12])/2).astype(int)
    half[[7,11]]=distal[[7,11]]
    stages=[('transfer_thumb_wrist_and_tips',distal),('grip_index_middle_seat',half),
            ('grip_lower_push',raised),('fixed_grip_close',grip)]
    rows=list(prior['goals'][:prefix_count]);phases=list(prior['phases'][:prefix_count].astype(str))
    previous=pushed;ends={};targets={}
    for phase,target in stages:
        part=interpolate(previous,target);rows.extend(part);phases.extend([phase]*len(part))
        ends[phase]=len(rows)-1;targets[phase]=target.tolist();previous=target.copy()
    rows.extend([grip.copy()]*134);phases.extend(['fixed_grip_hold']*134)
    goals=np.array(rows,dtype=int);validate_rows(prior['start'],goals)
    if not np.array_equal(goals[:prefix_count],prior['goals'][:prefix_count]):raise RuntimeError('Observed prefix changed')
    if not np.array_equal(goals[-1],grip):raise RuntimeError('Final grip changed')
    OUT.mkdir(exist_ok=True)
    np.savez_compressed(OUT/'candidate.npz',start=prior['start'],goals=goals,phases=np.array(phases,dtype='<U64'))
    report=dict(kind='overlapped_thumb_retreat_index_middle_distal_first_then_lower_push',
        total_goals=len(goals),preserved_prefix_goals=prefix_count,first1417_exactly_preserved=True,
        fixed_grip_exactly_preserved=True,phase_end_indices=ends,stage_target_ticks=targets,
        recorded_fourth_pose_motor_ids_used=list(range(5,13)),thumb_ring_from_fourth_pose_not_used=True,
        inward_native_limit_adjustments=adjustments,new_joint_limits_added=False,
        lower_push_moves_only_index_middle_non_distal_ids=[6,7,8,10,11,12],
        tip_ids_held_during_seating_and_lower_push=[5,9],
        proximal_fraction_before_base_push=.5,
        maximum_goal_step_ticks=2,movement_period_s=.075,cube_transfer_physically_verified=False,
        candidate_sha256=sha(OUT/'candidate.npz'),fourth_reference_sha256=sha(refpath),
        prior_candidate_sha256=sha(BASE/'taught_palm_transfer/candidate.npz'))
    (OUT/'construction_review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False))

if __name__=='__main__':main()
