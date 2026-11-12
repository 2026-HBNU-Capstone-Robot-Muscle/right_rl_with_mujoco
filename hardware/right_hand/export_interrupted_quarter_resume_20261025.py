"""Bind continuation to verified readbacks, never the old planned-final field."""
import json,hashlib
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/reverse_quarter_resume_empty'

def main():
    OUT.mkdir(exist_ok=True)
    trial=ROOT/'reverse_quarter_path_empty_20261025_082749_355962/result.json';r=json.loads(trial.read_bytes())
    assert r['all_torque_off_verified'] and r['settings_restored'] and r['samples']
    last=r['samples'][-1];assert last['goal']==last['goal_readback']
    source=ROOT/'right_hand_rotation_20261024/reverse_quarter_path_empty/quarter_turn_goals_v1.json'
    full=np.array(json.loads(source.read_bytes())['raw_goal_ticks']);matches=np.flatnonzero(np.all(full==last['goal'],axis=1))
    if len(matches)!=1:raise ValueError('Stopped readback does not identify one original waypoint')
    index=int(matches[0]);remaining=full[index:]
    fresh=ROOT/'reverse_quarter_path_empty_20261025_082933_354374/fresh_off_request.json'
    fresh_result=json.loads(fresh.with_name('result.json').read_bytes());assert fresh_result['write_events']==[] and fresh_result['off_start_endpoint_stable'] and fresh_result['all_torque_off_verified']
    off=json.loads(fresh.read_bytes())['off_state']
    data=dict(raw_goal_ticks=remaining.tolist(),motor_ids=list(range(1,17)),sample_period_seconds=.05,
        source_full_goals=source.relative_to(ROOT).as_posix(),source_full_goals_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        source_actual_trial=trial.relative_to(ROOT).as_posix(),source_actual_trial_sha256=hashlib.sha256(trial.read_bytes()).hexdigest(),
        resumed_original_waypoint=index,verified_last_issued_goal=last['goal'],planned_final_command_not_used=True,
        complete_original_waypoint_count=len(full),remaining_motion_seconds=(len(remaining)-1)*.05,
        loaded_cube_execution_allowed=False,physical_cube_rotation_verified=False)
    (OUT/'resume_goals_v1.json').write_text(json.dumps(data,indent=2),encoding='utf-8')
    (OUT/'actual_off_source.json').write_text(json.dumps(dict(source=fresh.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(fresh.read_bytes()).hexdigest(),off_state=off),indent=2),encoding='utf-8')
    print('RESUME_INDEX',index,'remaining_goals',len(remaining)-1,'remaining_seconds',data['remaining_motion_seconds'])
    print('LAST_VERIFIED_COMMAND',last['goal']);print('CURRENT_OFF',off['position'])

if __name__=='__main__':main()
