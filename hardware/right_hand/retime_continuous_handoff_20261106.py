"""Shorten only repeated internal goals, then replay every1ms from placement.

Final10.05s hold, all moving goals, geometry, force caps and safety limits
are preserved. A shorter array alone is not evidence of a working motion.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json,time
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT,R
from canonical_grip_dynamics_20261105 import run
from research_rotation_body_hold_20261104 import restore
from checkpoint_quarter_review_20261106 import quarter_reasons
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg
from evaluate_canonical_grip_reaction_20261105 import evaluate

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--keep',type=int,choices=(20,40),required=True);a=ap.parse_args();parent=ROOT/'continuous_handoff';review=json.loads((parent/'review.json').read_bytes());path=parent/'candidate.npz';source=np.load(path);allrows=source['goals'];selection=[];i=0
    while i<len(allrows):
        end=i+1
        while end<len(allrows) and np.array_equal(allrows[end],allrows[i]):end+=1
        count=end-i;selection.extend(range(i,end if end==len(allrows) or count<=40 else i+min(a.keep,count)));i=end
    rows=allrows[selection];assert np.max(abs(np.diff(np.vstack([source['start'],rows]),axis=0)))<=2 and np.array_equal(rows[-134:],np.tile(rows[-1],(134,1)))
    model=R/review['source_model'];initial=R/review['source_initial_state'];m=mujoco.MjModel.from_binary_path(str(model));z=np.load(initial);out=ROOT/'continuous_retiming'/f'keep{a.keep}';out.mkdir(parents=True,exist_ok=True);np.savez_compressed(out/'candidate.npz',start=source['start'],goals=rows,source_goal_indices=selection);began=time.perf_counter();v=run(m,z,rows,1,out/'end_state.npz',out/'trajectory.npz',True,5,('if_tip','th_tip'));end=restore(m,np.load(out/'end_state.npz'));target=upright_yaw_target(z['initial_cube_rotation'],z['up'],90);error=orientation_error_deg(end.xmat[m.body('cube').id].reshape(3,3),target);why=quarter_reasons(v,review['planned_nominal_common_center_world_m'],True,error);report=dict(status='internal_constant_goal_retiming_full_initial_replay',source_candidate=path.relative_to(R).as_posix(),source_candidate_sha256=sha(path),source_initial_state=initial.relative_to(R).as_posix(),source_initial_state_sha256=sha(initial),source_model=model.relative_to(R).as_posix(),source_model_sha256=sha(model),keep_internal_constant_goals=a.keep,total_goals=len(rows),removed_constant_goals=len(allrows)-len(rows),nominal_seconds=len(rows)*.075,candidate_sha256=sha(out/'candidate.npz'),all_moving_goals_and_final134_constant_goals_preserved=True,dynamic=v,orientation_error_deg=error,quarter_and_handoff_passed=not why,reasons=why,all_cube_dynamics_continuous_no_resets=True,original_goal_step_and_dynamic_bounds_unchanged=True,devices_opened=False,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began);(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(keep=a.keep,goals=len(rows),seconds=len(rows)*.075,safe=v['safety_passed'],passed=not why,yaw=v['final_yaw_from_placement_deg'],reasons=why)),flush=True)
    if not why:
        loads=[];state=dict(np.load(out/'end_state.npz'));state['required_lower_contacts']=np.asarray(['if_tip','th_tip'])
        for torque in (0.,-.003,.003,-.01,.01):
            c=evaluate(m,state,torque,out/f'load{torque:+g}.npz');loads.append(c);print(json.dumps(dict(load=torque,passed=c['load_filter_passed'],reasons=c['reasons'])),flush=True)
        report['hypothetical_load_cases']=loads;report['all_nominal_hypothetical_loads_passed']=all(c['load_filter_passed'] for c in loads);(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
