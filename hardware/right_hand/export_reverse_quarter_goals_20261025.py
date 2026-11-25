"""Export proposed, not measured, goals and a separately sourced OFF fixture."""
import json,hashlib
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
MODEL=ROOT/'right_hand_rotation_20261024/reverse_empty_entry/actual_075405_loaded_model'
SOURCE=MODEL/'tip_clearance_search_v3/candidate_g1_stop38_tip-24_ring-56.npz'
OUT=ROOT/'right_hand_rotation_20261024/reverse_quarter_path_empty'

def main():
    OUT.mkdir(exist_ok=True)
    raw=np.load(SOURCE)['goal_ticks'];limits=json.loads((ROOT/'right_hand_rotation_20261023/native_motor_limits.json').read_bytes())
    assert raw.shape[1]==16 and np.max(abs(np.diff(raw,axis=0)))<=2
    assert np.all(raw>=np.array(limits['lower_ticks'])) and np.all(raw<=np.array(limits['upper_ticks']))
    proposed=dict(source_candidate=str(SOURCE.relative_to(ROOT)),source_candidate_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        raw_goal_ticks=raw.tolist(),motor_ids=list(range(1,17)),sample_period_seconds=.05,
        nominal_motion_seconds=(len(raw)-1)*.05,measured_trajectory=False,
        loaded_cube_execution_allowed=False,independent_empty_runtime_screen_required=True,physical_cube_rotation_verified=False,
        note='Proposed hand path for EMPTY testing only. Nominal loaded model passes; weak-gain loaded rotation is not established.')
    p=OUT/'quarter_turn_goals_v1.json';p.write_text(json.dumps(proposed,indent=2),encoding='utf-8')
    trial=ROOT/'reverse_start_placement_20261025_075405_164917/result.json';r=json.loads(trial.read_bytes())
    assert r['final_off_reference_valid'] and r['cube_removal_confirmed'] and r['all_torque_off_verified']
    off=r['final_off_snapshots'][-1]
    print('OFF_KEYS',list(off))
    if 'position' not in off:raise ValueError('Inspect OFF snapshot schema before creating a fixture')
    state={k:off[k] for k in ('position','torque','error','watchdog','voltage','temperature')}
    print('ACTUAL_OFF_POSITION',state['position'])
    (OUT/'actual_off_source.json').write_text(json.dumps(dict(source=str(trial.relative_to(ROOT)),source_sha256=hashlib.sha256(trial.read_bytes()).hexdigest(),
        measured_off_state=state),indent=2),encoding='utf-8')
    print('GOALS',len(raw),'nominal_motion_seconds',proposed['nominal_motion_seconds'],'sha256',hashlib.sha256(p.read_bytes()).hexdigest())

if __name__=='__main__':main()
