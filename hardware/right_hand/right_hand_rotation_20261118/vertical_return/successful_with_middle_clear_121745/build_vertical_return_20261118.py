"""Pitch-end -> executed palm transfer -> checked receiving -> observed G tail."""
from pathlib import Path
import hashlib,json
import numpy as np
import leap_measured_grip_hold_20261117 as pose
from build_taught_palm_transfer_20261118 import interpolate
from measured_grip_small_cycle_plan_20261117 import validate_rows

ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261118/vertical_return'
VERTICAL=ROOT/'right_hand_rotation_20261118/vertical_cycle/candidate.npz'
TRANSFER=ROOT/'right_hand_rotation_20261028/two_axis_connection/operator_pitch_handoff_v3_side_clearance/transfer_path.npz'
GRIP_RETURN=ROOT/'right_hand_rotation_20261118/grip_rotation_return/candidate.npz'
TAUGHT_PALM=BASE/'taught_palm_121158.json'

def main():
    v=np.load(VERTICAL,allow_pickle=False);t=np.load(TRANSFER,allow_pickle=False);g=np.load(GRIP_RETURN,allow_pickle=False)
    old_run=json.loads((ROOT/'pitch_handoff_observe_20261028_221727_181336/result.json').read_bytes())
    actual=[s['goal'] for s in old_run['samples'] if s['phase']=='stage_7_loaded_pose_transfer']
    if not np.array_equal(actual,np.vstack([t['start'],t['goals'][:-1]])):raise RuntimeError('Saved loaded transfer differs from actual command evidence')
    grip_run=json.loads((ROOT/'grip_rotation_return_trial_20261118_102359_147864/result.json').read_bytes())
    if (grip_run['source_candidate_sha256']!=hashlib.sha256(GRIP_RETURN.read_bytes()).hexdigest()
        or grip_run['completed_goals']!=grip_run['planned_goals'] or not grip_run['holding']):raise RuntimeError('Successful grip return source changed')
    start=v['goals'][-1].copy();align=interpolate(start,t['start'])
    clear_end=int(np.flatnonzero(g['phases']=='return_transfer_middle_clear')[-1])
    prepare_end=int(np.flatnonzero(g['phases']=='return_transfer_thumb_prepare')[-1])
    push_start=int(np.flatnonzero(g['phases']=='return_transfer_thumb_push')[0])
    taught=json.loads(TAUGHT_PALM.read_bytes())
    raw=ROOT/taught['source']
    if hashlib.sha256(raw.read_bytes()).hexdigest()!=taught['source_sha256']:raise RuntimeError('Taught palm source changed')
    palm_target=np.asarray(taught['goal_ticks'],dtype=int)
    palm_align=interpolate(t['goals'][-1],palm_target)
    cleared=palm_target.copy();cleared[8:12]=g['goals'][clear_end,8:12]
    middle_clear=interpolate(palm_target,cleared)
    thumb_prepare=interpolate(cleared,g['goals'][prepare_end])
    tail=g['goals'][push_start:]
    goals=np.vstack([align,t['goals'],palm_align,middle_clear,thumb_prepare,tail]);validate_rows(start,goals)
    if not np.array_equal(goals[-1],pose.reference()[2]):raise RuntimeError('Vertical return ends at a different G')
    phases=np.concatenate([np.array(['pitch_side_clearance']*len(align)),
        np.array(['pitch_palm_transfer']*len(t['goals'])),np.array(['taught_palm_receiving']*len(palm_align)),
        np.array(['middle_clear_for_push']*len(middle_clear)),
        np.array(['thumb_prepare_for_center']*len(thumb_prepare)),g['phases'][push_start:]])
    BASE.mkdir(exist_ok=True)
    np.savez_compressed(BASE/'candidate.npz',start=start,goals=goals,phases=phases)
    review=dict(kind='vertical_return_only_staged_observation',goals=len(goals),start_ticks=start.tolist(),
        final_grip_ticks=goals[-1].tolist(),candidate_sha256=hashlib.sha256((BASE/'candidate.npz').read_bytes()).hexdigest(),
        alignment_goals=len(align),historical_loaded_transfer_goals=len(t['goals']),
        taught_palm_receiving_goals=len(palm_align),taught_palm_target_ticks=palm_target.tolist(),
        taught_palm_reference=TAUGHT_PALM.relative_to(ROOT).as_posix(),
        middle_clear_goals=len(middle_clear),thumb_prepare_goals=len(thumb_prepare),
        observed_fixed_grip_tail_goals=len(tail),shared_tail_source_offset=push_start,
        thumb_centering_push_included=True,centering_target='between_index_and_middle',
        historical_transfer_command_match=True,shared_G_tail_exact=True,physical_vertical_return_verified=False,
        original_forward_pitch_unchanged=True,new_joint_limits_added=False,control_period_seconds=.075)
    (BASE/'construction_review.json').write_text(json.dumps(review,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(review))

if __name__=='__main__':main()
