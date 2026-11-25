"""Join the physically observed vertical forward and return without a reset."""
from pathlib import Path
import hashlib,json
import numpy as np
import leap_measured_grip_hold_20261117 as pose
from measured_grip_small_cycle_plan_20261117 import validate_rows
from vertical_transfer_plan_20261118 import VerticalTransfer
from vertical_return_plan_20261118 import VerticalReturn

ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261118/vertical_rotation_return'
EVIDENCE=('vertical_transfer_trial_20261118_112445_334008/result.json',
          'vertical_return_trial_20261118_123353_142977/result.json')

def main():
    forward=VerticalTransfer(force_grip=True);returned=VerticalReturn()
    proofs=[]
    for name,sequence in zip(EVIDENCE,(forward,returned)):
        r=json.loads((ROOT/name).read_bytes())
        if (r['source_candidate_sha256']!=sequence.manifest['candidate_sha256']
            or not r['holding'] or r['completed_goals']!=r['planned_goals']
            or not r['all_torque_off_verified'] or not r['settings_restored']):
            raise RuntimeError('Successful vertical source evidence changed: '+name)
        proofs.append(dict(source=name,sha256=hashlib.sha256((ROOT/name).read_bytes()).hexdigest(),
            completed_goals=r['completed_goals']))
    if not np.array_equal(forward.goals[-1],returned.start):raise RuntimeError('Vertical join target differs')
    hold=returned.phases=='return_fixed_grip_hold'
    first_hold=int(np.flatnonzero(hold)[0])
    if not np.all(hold[first_hold:]) or not np.all(returned.goals[first_hold:]==forward.start):
        raise RuntimeError('Only the final constant G hold may be omitted')
    # The same G remains commanded by the runtime final hold. Do not change
    # any moving target, speed or hardware-health guard.
    goals=np.vstack([forward.goals,returned.goals[:first_hold]])
    phases=np.concatenate([np.array(['cycle_forward_'+str(p) for p in forward.phases]),
                           np.array(['cycle_return_'+str(p) for p in returned.phases[:first_hold]])])
    validate_rows(forward.start,goals)
    BASE.mkdir(exist_ok=True)
    np.savez_compressed(BASE/'candidate.npz',start=forward.start,goals=goals,phases=phases)
    sources={}
    for seq in (forward,returned):
        for name,digest in seq.manifest['source_hashes'].items():
            if name in sources and sources[name]!=digest:raise RuntimeError('Conflicting source contract: '+name)
            sources[name]=digest
    own=('build_vertical_rotation_return_20261118.py','vertical_rotation_return_plan_20261118.py',
         'leap_vertical_rotate_grip_20261118.py','leap_vertical_rotate_grip.cmd','test_vertical_rotation_return_20261118.py',
         'right_hand_rotation_20261118/vertical_cycle/candidate.npz',
         'right_hand_rotation_20261118/vertical_cycle/operator_trial_manifest.json',
         'right_hand_rotation_20261118/vertical_return/candidate.npz',
         'right_hand_rotation_20261118/vertical_return/operator_trial_manifest.json',
         'vertical_rotate_grip_trial_20261118_124856_325299/result.json',
         'vertical_rotate_grip_trial_20261118_195332_084896/result.json',*EVIDENCE)
    sources.update({name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in own})
    manifest=dict(kind='observed_vertical_rotation_then_return_same_G',goals=len(goals),
        candidate_sha256=hashlib.sha256((BASE/'candidate.npz').read_bytes()).hexdigest(),
        source_hashes=sources,source_evidence=proofs,forward_goals=len(forward.goals),return_goals=first_hold,
        final_constant_hold_rows_replaced_by_runtime_hold=int(hold.sum()),
        join_endpoint_mismatch_ticks=int(np.max(abs(forward.goals[-1]-returned.start))),
        join_step_ticks=int(np.max(abs(returned.goals[0]-forward.goals[-1]))),
        all_moving_goals_preserved=True,final_grip_ticks=forward.start.tolist(),
        operator_gated_observation_enabled=True,physical_combined_execution_verified=False,
        automatic_repetition_included=False,maximum_ON_seconds=None,maximum_step_ticks=2,
        period_seconds=.075,current_cap_ma=450,PID_unchanged=True,tests_passed=0,
        elapsed_time_limit_enabled=False,user_authorized_elapsed_time_limit_removal=True,
        live_remaining_time_checked_before_each_advance=False)
    (BASE/'operator_trial_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in manifest.items() if k not in ('source_hashes','source_evidence')},ensure_ascii=False))

if __name__=='__main__':main()
