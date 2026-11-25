"""Join the observed G->turn to the previously observed turn->same-G return."""
from pathlib import Path
import hashlib,json
import numpy as np
import leap_measured_grip_hold_20261117 as pose
from retaught_transfer_scope_20261118 import limits
from measured_grip_small_cycle_plan_20261117 import validate_rows
from build_taught_palm_transfer_20261118 import interpolate

ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261118/grip_rotation_return'
FORWARD=ROOT/'right_hand_rotation_20261118/grip_to_rotation/retaught_093413/candidate.npz'
RETURN=ROOT/'right_hand_rotation_20261118/quarter_turn_fixed_grip/tip_first_lower_seat/candidate.npz'

def main():
    f=np.load(FORWARD,allow_pickle=False);r=np.load(RETURN,allow_pickle=False)
    forward_run=json.loads((ROOT/'retaught_transfer_trial_20261118_094439_457156/result.json').read_bytes())
    return_run=json.loads((ROOT/'quarter_fixed_grip_trial_20261118_064459_264578/result.json').read_bytes())
    for path,run in ((FORWARD,forward_run),(RETURN,return_run)):
        if (run['source_candidate_sha256']!=hashlib.sha256(path.read_bytes()).hexdigest()
            or run['completed_goals']!=run['planned_goals'] or not run['holding']):
            raise RuntimeError('Completed physical source run does not match its candidate')
    if not np.array_equal(f['goals'][-1],r['goals'][1416]):raise RuntimeError('Return start differs from observed turn end')
    grip=pose.reference()[2];original=r['goals'][1417:];original_phases=r['phases'][1417:]
    seat=int(np.flatnonzero(original_phases=='grip_index_middle_seat')[0])
    root_before=int(original[seat-1,11]);root_early=root_before+32
    if root_early>grip[11]:raise RuntimeError('Root lead exceeds already-used final root target')
    lifted=original[seat-1].copy();lifted[11]=root_early
    lead=interpolate(original[seat-1],lifted)
    changed=original.copy();changed[seat:,11]=np.maximum(changed[seat:,11],root_early)
    returned=np.vstack([original[:seat],lead,changed[seat:]])
    return_phases=np.concatenate([original_phases[:seat],np.array(['middle_root_lead']*len(lead)),original_phases[seat:]])
    goals=np.vstack([f['goals'],returned])
    if not np.array_equal(goals[-1],grip):raise RuntimeError('Final fixed grasp changed')
    with limits():validate_rows(grip,goals)
    BASE.mkdir(exist_ok=True)
    np.savez_compressed(BASE/'candidate.npz',start=grip,goals=goals,
        phases=np.concatenate([np.array(['forward_'+str(p) for p in f['phases']]),
                               np.array(['return_'+str(p) for p in return_phases])]))
    review=dict(kind='observed_grip_turn_and_observed_return_join',forward_goals=len(f['goals']),
        return_source_offset=1417,return_goals=len(returned),original_return_goals=len(original),goals=len(goals),
        forward_exact=True,return_exact=False,return_other_15_joints_exact=True,
        root_lead=dict(motor_id=12,ticks=32,degrees=32*360/4096,from_ticks=root_before,to_ticks=root_early,
                       before_source_return_index=seat,extra_goal_count=len(lead)),
        join_goal_step_ticks=int(np.max(abs(returned[0]-f['goals'][-1]))),
        final_grip_ticks=grip.tolist(),candidate_sha256=hashlib.sha256((BASE/'candidate.npz').read_bytes()).hexdigest(),
        physical_join_verified=False,automatic_repetition_included=False,maximum_ON_seconds=500,
        control_period_seconds=.075,maximum_goal_step_ticks=2)
    (BASE/'construction_review.json').write_text(json.dumps(review,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(review))

if __name__=='__main__':main()
