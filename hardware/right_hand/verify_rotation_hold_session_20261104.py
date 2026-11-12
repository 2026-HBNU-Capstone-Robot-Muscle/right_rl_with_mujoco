"""Meaningful invariants for research paths, evidence scope and sweep geometry."""
import json
import numpy as np
from research_rotation_body_hold_20261104 import R,OUT
from loaded_quarter_reentry_20261025 import limits
from audit_upper_layer_sweep_20261104 import sweep

def box(c,h):
    return np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])*np.array(h)+np.array(c)

def main():
    checks={};lo,hi=limits();z=np.load(OUT/'body_hold_research_candidate.npz');values=np.vstack([z['start'],z['goals']]);source=np.load(R/'right_hand_rotation_20261104/raised_cradle_160_yaw0/candidate.npz')
    checks['same_recorded_prefix']=np.array_equal(z['goals'][:1368],source['goals'][:1368])
    checks['all_new_goals_native']=bool(np.all(values>=lo) and np.all(values<=hi))
    checks['max2tick_goal_steps']=bool(np.max(abs(np.diff(values,axis=0)))<=2)
    checks['final5s_goals_fixed']=bool(np.all(z['goals'][-67:]==z['goals'][-1]))
    checks['lower_contact_suffix_preserves_index_ring']=bool(np.all(z['goals'][1368:,[4,5,6,7,12,13,14,15]]==source['goals'][1367,[4,5,6,7,12,13,14,15]]))
    checks['sweep_accepts_vertical_clearance']=sweep(box([0,0,0],[1,1,1]))['continuous_sweep_separation_proved']
    checks['sweep_rejects_initial_overlap']=sweep(box([27,-55/3,0],[.5,.5,.5]))['sampled_overlap_predicted']
    corner=sweep(box([34.5,-55/3,0],[.5,.5,.5]))
    checks['sweep_catches_corner_between_endpoints']=corner['sampled_overlap_predicted'] and not corner['continuous_sweep_separation_proved']
    summary=json.loads((OUT/'session_summary.json').read_bytes())
    checks['no_operator_admission']=not summary['hardware_execution_allowed'] and summary['next_user_motion_command'] is None
    checks['physical_claims_remain_unverified']=not summary['whole_cube90_verified'] and not summary['physical_hold_verified']
    checks['partial_replay_failure_preserved']=summary['continuous_replay_passed']==8 and summary['initial_seating_failures']==1
    checks['whole_hand_interference_preserved']=not summary['whole_collision_hand_upper_sweep_clear'] and bool(summary['remaining_sweep_geometries'])
    report=dict(checks={k:bool(v) for k,v in checks.items()},passed=all(checks.values()),devices_opened=False)
    (OUT/'verification.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report,indent=2));assert report['passed']
if __name__=='__main__':main()
