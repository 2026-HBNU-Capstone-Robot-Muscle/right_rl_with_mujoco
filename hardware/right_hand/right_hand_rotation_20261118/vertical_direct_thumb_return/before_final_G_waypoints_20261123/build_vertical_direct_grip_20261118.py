"""Recorded vertical turn with a short taught thumb return, offline only."""
from pathlib import Path
import hashlib,json
import numpy as np
import leap_measured_grip_hold_20261117 as pose
from vertical_transfer_plan_20261118 import VerticalTransfer
from measured_grip_small_cycle_plan_20261117 import validate_rows
from build_taught_palm_transfer_20261118 import interpolate

ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261118/vertical_direct_thumb_return'
RECEIVING=ROOT/'right_hand_rotation_20261118/vertical_return/taught_palm_121158.json'
REFS=('thumb_push_reference.json','thumb_clear_reference.json')

def grip_entry(current,grip):
    current=pose.encoder_vector(current);_,high,low=pose.bounds()
    inward=np.clip(current,np.ceil(low).astype(int),high)
    chunks=[pose.path(current,inward)];phases=['direct_entry_range_return']*len(chunks[0]);q=inward.copy()
    # Lower the thumb into G before the other fingers push/close around it.
    for name,ids in [('direct_entry_thumb_lower',list(range(4))),('direct_entry_tips',[4,8,12]),('direct_entry_support',list(range(4,16)))]:
        target=q.copy();target[ids]=grip[ids];part=interpolate(q,target)
        chunks.append(part);phases.extend([name]*len(part));q=target
    rows=np.vstack(chunks);validate_rows(current,rows)
    return rows,phases

def prepare_manifest():
    forward=VerticalTransfer(force_grip=True)
    review=json.loads((BASE/'construction_review.json').read_bytes())
    static=json.loads((BASE/'static_review.json').read_bytes())
    digest=hashlib.sha256((BASE/'candidate.npz').read_bytes()).hexdigest()
    if review['candidate_sha256']!=digest or static['candidate_sha256']!=digest:
        raise RuntimeError('Direct candidate/static review mismatch')
    if any(static[k] for k in ['added_self_mm','added_support_mm','entry_added_self_mm','entry_added_support_mm']):
        raise RuntimeError('New static interference remains')
    sources=dict(forward.manifest['source_hashes'])
    names=['build_vertical_direct_grip_20261118.py','vertical_direct_grip_plan_20261118.py',
        'leap_vertical_direct_grip_20261118.py','leap_vertical_direct_grip.cmd','test_vertical_direct_grip_20261118.py',
        'screen_vertical_direct_grip_20261118.py','right_hand_rotation_20261118/vertical_cycle/candidate.npz',
        'right_hand_rotation_20261118/vertical_cycle/operator_trial_manifest.json',
        'right_hand_rotation_20261118/vertical_rotation_return/candidate.npz',
        'vertical_direct_grip_trial_20261118_204744_702191/result.json',RECEIVING.relative_to(ROOT).as_posix()]
    for name in REFS:
        ref=BASE/name;names.append(ref.relative_to(ROOT).as_posix())
        names.append(json.loads(ref.read_bytes())['source'])
    names.append((BASE/'static_review.json').relative_to(ROOT).as_posix())
    sources.update({name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in names})
    review.update(source_hashes=sources,operator_gated_observation_enabled=True,tests_passed=0)
    (BASE/'operator_trial_manifest.json').write_text(json.dumps(review,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print('Direct trial manifest prepared; physical return still unverified.')

def main():
    forward=VerticalTransfer(force_grip=True);start=forward.goals[-1].copy();grip=pose.reference()[2]
    refs=[json.loads((BASE/name).read_bytes()) for name in REFS]
    for ref in refs:
        if hashlib.sha256((ROOT/ref['source']).read_bytes()).hexdigest()!=ref['source_sha256']:
            raise RuntimeError('Direct thumb source changed')
    receiving=json.loads(RECEIVING.read_bytes())
    q=start.copy();q[4:12]=np.array(receiving['goal_ticks'])[4:12]
    waypoints=[('lower_index_middle_receiving',q.copy())]
    q[:4]=refs[0]['holding_goal_ticks'];waypoints.append(('thumb_push_toward_ring',q.copy()))
    q[:4]=refs[1]['holding_goal_ticks'];waypoints.append(('thumb_clear_cube',q.copy()))
    # The ring must clear with the thumb; holding its pitch-end shape here
    # creates a thumb/ring overlap. Index and middle still remain stationary.
    q[:4]=grip[:4];q[12:]=grip[12:];waypoints.append(('thumb_lower_before_support',q.copy()))
    q[[4,8,12]]=grip[[4,8,12]];waypoints.append(('finger_tips_first',q.copy()))
    q[4:]=grip[4:];waypoints.append(('lower_support_close',q.copy()))
    rows=[];phases=[];previous=start
    for name,target in waypoints:
        segment=interpolate(previous,target);rows.extend(segment);phases.extend([name]*len(segment));previous=target
    returned=np.asarray(rows,dtype=int);validate_rows(start,returned)
    goals=np.vstack([forward.goals,returned])
    labels=np.concatenate([np.array(['direct_forward_'+str(p) for p in forward.phases]),
                           np.array(['direct_return_'+p for p in phases])])
    validate_rows(forward.start,goals)
    np.savez_compressed(BASE/'candidate.npz',start=forward.start,goals=goals,phases=labels)
    review=dict(kind='vertical_turn_taught_direct_thumb_return',candidate_sha256=hashlib.sha256((BASE/'candidate.npz').read_bytes()).hexdigest(),
        goals=len(goals),forward_goals=len(forward.goals),return_goals=len(returned),
        stage_target_ticks={name:q.tolist() for name,q in waypoints},
        phase_goals={name:int(phases.count(name)) for name,_ in waypoints},
        all_forward_goals_preserved=True,fixed_G_unchanged=True,thumb_reference_motor_ids=[1,2,3,4],
        taught_whole_hand_poses_not_replayed=True,other12_held_during_thumb_push_and_clear=True,
        grip_entry_order=['thumb_lower','finger_tips','lower_support'],
        final_grip_order=['thumb_clear','thumb_lower','finger_tips','lower_support'],
        thumb_G_target_before_other_finger_push=True,
        ring_coordinates_with_return_thumb_lowering=True,
        old600_goal_palm_transfer_removed=True,elapsed_time_limit_enabled=False,
        maximum_ON_seconds=None,maximum_step_ticks=2,period_seconds=.075,current_cap_ma=450,
        PID_unchanged=True,new_joint_limits_added=False,physical_direct_return_verified=False)
    (BASE/'construction_review.json').write_text(json.dumps(review,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(review,ensure_ascii=False))

if __name__=='__main__':
    import sys
    prepare_manifest() if '--prepare-manifest' in sys.argv else main()
