"""Build a bounded middle-only probe between the physically reached grip/open.

The thumb remains at its reached grip throughout the probe. This measures the
effect of clearing the middle finger; it does not claim a completed lower grip.
"""
import hashlib,json
from pathlib import Path
import numpy as np
from loaded_quarter_reentry_20261025 import limits
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261106/middle_clearance_probe'
NAT=R/'right_hand_rotation_20261030/natural_cradle_grasp/candidate.npz'
TEACH=R/'manual_grasp_teach_20261106_183247_578628/result.json'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def ramp(a,b):
    count=max(1,int(np.ceil(max(abs(b-a))/2)))
    return np.rint([a+(b-a)*i/count for i in range(1,count+1)]).astype(int)
def build():
    O.mkdir(exist_ok=True);z=np.load(NAT);t=json.loads(TEACH.read_bytes());old=z['goals'];phases=z['phases'];middle=np.arange(8,12);grip=old[926].copy();goal=grip.copy();goal[middle]+=np.rint(.5*(np.array(t['corrected_position_ticks'])[middle]-np.array(t['last_powered_position_ticks'])[middle])).astype(int)
    forward=ramp(grip,goal);back=ramp(goal,grip);blocks=[old[:927],forward,np.tile(goal,(67,1)),back,np.tile(grip,(20,1)),old[927:]];p=np.r_[phases[:927],['stage_7_middle_fold']*len(forward),['stage_8_middle_hold']*67,['stage_8b_middle_return']*len(back),['stage_8c_restored_hold']*20,phases[927:]];rows=np.vstack(blocks);lo,hi=limits()
    assert np.all(rows>=lo) and np.all(rows<=hi) and max(abs(np.diff(np.vstack([z['start'],rows]),axis=0)).ravel())<=2
    fixed=np.r_[np.arange(8),np.arange(12,16)];assert np.all(forward[:,fixed]==grip[fixed]) and np.all(back[:,fixed]==grip[fixed]);assert np.array_equal(rows[:927],old[:927]) and np.array_equal(rows[-len(old[927:]):],old[927:])
    candidate=O/'candidate.npz';np.savez_compressed(candidate,start=z['start'],goals=rows,phases=p)
    aliases={'stage_7_middle_fold':'stage_7_horizontal_yaw','stage_8_middle_hold':'stage_8_horizontal_hold','stage_8b_middle_return':'stage_7_horizontal_yaw','stage_8c_restored_hold':'stage_8_horizontal_hold'}
    modelp=np.array([aliases.get(str(v),str(v)) for v in p]);end=927+len(forward)+67;np.savez_compressed(O/'evaluation_alias.npz',start=z['start'],goals=rows,phases=modelp,forward_end_sample=end)
    report=dict(status='middle_only_diagnostic_candidate_not_yet_admitted',candidate_sha256=sha(candidate),evaluation_alias_sha256=sha(O/'evaluation_alias.npz'),source_natural_sha256=sha(NAT),source_teaching_sha256=sha(TEACH),source_script_sha256=sha(Path(__file__)),middle_tip_to_root_ids=[9,11,10,12],middle_motor_order_ids=[9,10,11,12],old_grip=grip.tolist(),middle_probe_goal=goal.tolist(),source_curvature_fraction=.5,correction_formula='old grip goal + half (saved user bend minus last powered actual), middle joints only',thumb_index_ring_fixed_during_probe=True,existing_grasp_and_release_goal_vectors_unchanged=True,goals=len(rows),loaded_nominal_seconds=len(rows)*.075,probe_goals=len(forward),probe_seconds=len(forward)*.075,probe_hold_seconds=67*.075,phase_aliases_are_evaluator_compatibility_only=True,no_motor_yaw_or_cube_rotation_target=True,evaluation_aliases=aliases,new_common_grip_verified=False,physical_success=False,hardware_execution_allowed=False,devices_opened=False)
    (O/'proposal.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report),flush=True)
if __name__=='__main__':build()
