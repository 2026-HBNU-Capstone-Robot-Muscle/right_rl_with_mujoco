"""Replay the entire placement-to-handoff lineage without intermediate resets.

Every saved suffix must start at the preceding segment's exact last goal.
Fresh free-cube dynamics, all original1ms guards and final upper-space/load
proofs remain required. This script has no device or live-motion interface.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import hashlib,json,time
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT,R
from canonical_grip_dynamics_20261105 import run
from research_rotation_body_hold_20261104 import restore
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg
from checkpoint_quarter_review_20261106 import quarter_reasons
from evaluate_canonical_grip_reaction_20261105 import evaluate

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    second=ROOT/'supported_handoff_search_mf_active_second';r=json.loads((second/'review.json').read_bytes());passed=[v for v in r['full1ms_replays'] if v['checkpoint90_and_grip_passed']];assert passed
    verified=second/'verified_generation5';assert (verified/'candidate.npz').is_file();model=ROOT/'torque_cap_sensitivity/cap0.15/model.mjb';initial=ROOT/'torque_cap_sensitivity/cap0.15/initial_state.npz';m=mujoco.MjModel.from_binary_path(str(model));z=np.load(initial);out=ROOT/'continuous_handoff';out.mkdir(exist_ok=True);parts=[('quarter',ROOT/'near_quarter_full_replay/candidate.npz',ROOT/'near_quarter_full_replay/end_state.npz'),('middle_retreat',ROOT/'postturn_middle_clearance/candidate.npz',ROOT/'postturn_middle_clearance/end_state.npz'),('thumb_release',ROOT/'postturn_thumb_clearance/id1_delta-64/candidate.npz',ROOT/'postturn_thumb_clearance/id1_delta-64/end_state.npz'),('opposing_grasp',ROOT/'index_countergrip_workspace/p12_seeds6/zneg_0/force1.npz',ROOT/'index_countergrip_workspace/p12_seeds6/zneg_0/force1_end.npz'),('level',ROOT/'countergrip_level/seed0_source1_lift2_normal1.5_n850/suffix.npz',ROOT/'countergrip_level/seed0_source1_lift2_normal1.5_n850/verified_end_state.npz'),('lower_grip',ROOT/'countergrip_level/seed0_source1_lift0_normal2.5_n900_resume_lower_staged_freebase/suffix.npz',ROOT/'countergrip_level/seed0_source1_lift0_normal2.5_n900_resume_lower_staged_freebase/end_state.npz'),('fixed_baseline',None,ROOT/'temporary_handoff_load/torque+0.npz'),('first_correction',ROOT/'supported_handoff_search_mf_local/best_candidate.npz',ROOT/'supported_handoff_first_full/end_state.npz'),('second_correction',verified/'candidate.npz',verified/'end_state.npz')];segments=[];arrays=[];start=None;last=None;total=0;checkpoints={}
    for label,path,state in parts:
        if path is None:goals=np.tile(last,(134,1));part_start=last.copy()
        else:
            p=np.load(path);goals=p['goals'];part_start=p['start'];assert len(goals)>0
        if start is None:start=part_start.copy()
        else:assert np.array_equal(last,part_start),(label,last,part_start)
        arrays.append(goals);total+=len(goals);last=goals[-1].copy();checkpoints[total*75]=(label,state);segments.append(dict(label=label,source_goals=None if path is None else path.relative_to(R).as_posix(),source_goals_sha256=None if path is None else sha(path),source_saved_endpoint=state.relative_to(R).as_posix(),source_saved_endpoint_sha256=sha(state),goals=len(goals),cumulative_goals=total))
    rows=np.vstack(arrays);assert max(abs(np.diff(np.vstack([start,rows]),axis=0)).ravel())<=2;constant=np.array_equal(rows[-134:],np.tile(rows[-1],(134,1)));assert constant;np.savez_compressed(out/'candidate.npz',start=start,goals=rows);target_center=np.asarray(r['best_proposal']['nominal_terminal_center_world_m']);count=0;comparisons=[];original_step=mujoco.mj_step;began=time.perf_counter()
    def observed_step(model,data,*args,**kwargs):
        nonlocal count
        result=original_step(model,data,*args,**kwargs);count+=1
        if count in checkpoints:
            label,state=checkpoints[count];expected=np.load(state);comparisons.append(dict(label=label,step=count,maximum_qpos_error=float(np.max(abs(data.qpos-expected['qpos']))),maximum_qvel_error=float(np.max(abs(data.qvel-expected['qvel']))),maximum_ctrl_error=float(np.max(abs(data.ctrl-expected['ctrl'])))))
        if count%30000==0:print(json.dumps(dict(event='continuous_replay_progress',goals=count//75,total_goals=total,elapsed_seconds=time.perf_counter()-began)),flush=True)
        return result
    mujoco.mj_step=observed_step
    try:v=run(m,z,rows,1,out/'end_state.npz',out/'trajectory.npz',True,5,('if_tip','th_tip'))
    finally:mujoco.mj_step=original_step
    end=restore(m,np.load(out/'end_state.npz'));target=upright_yaw_target(z['initial_cube_rotation'],z['up'],90);error=orientation_error_deg(end.xmat[m.body('cube').id].reshape(3,3),target);why=quarter_reasons(v,target_center,constant,error);report=dict(status='continuous_initial_placement_to_new_handoff',source_model=model.relative_to(R).as_posix(),source_model_sha256=sha(model),source_initial_state=initial.relative_to(R).as_posix(),source_initial_state_sha256=sha(initial),segments=segments,total_goals=len(rows),nominal_seconds=len(rows)*.075,maximum_goal_increment_ticks=int(max(abs(np.diff(np.vstack([start,rows]),axis=0)).ravel())),candidate_sha256=sha(out/'candidate.npz'),dynamic=v,orientation_error_deg=error,planned_nominal_common_center_world_m=target_center.tolist(),quarter_and_handoff_passed=not why,reasons=why,source_checkpoint_state_comparisons=comparisons,all_goal_seams_exact=True,all_cube_dynamics_continuous_no_resets=True,upper_space_required_only_at_left_handoff=True,no_physical_policy_or_device_execution=True,devices_opened=False,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began);(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(event='continuous_replay_complete',safe=v['safety_passed'],passed=not why,yaw=v['final_yaw_from_placement_deg'],tilt=v['final_up_tilt_deg'],gap=v['settled_tail']['minimum_separating_plane_gap_1ms_mm'],fraction=v['settled_tail']['opposed_lower_body_contact_fraction'],reasons=why,goals=len(rows),seconds=time.perf_counter()-began)),flush=True)
    if not why:
        source_end=dict(np.load(out/'end_state.npz'));source_end['required_lower_contacts']=np.asarray(['if_tip','th_tip']);loads=[]
        for torque in (0.,-.003,.003,-.01,.01):
            load=evaluate(m,source_end,torque,out/f'load{torque:+g}.npz');loads.append(load);print(json.dumps(dict(event='continuous_endpoint_load',torque=torque,passed=load['load_filter_passed'],reasons=load['reasons'])),flush=True)
        report['hypothetical_load_cases']=loads;report['all_nominal_hypothetical_loads_passed']=all(c['load_filter_passed'] for c in loads);(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
