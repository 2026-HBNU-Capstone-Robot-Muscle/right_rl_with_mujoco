"""Slightly close the final side contacts while preserving cube pose.

The continuous path currently ends with about0.47N at each side. These
research targets are0.8/1.2N, not changes to real current limits or PID. Only
bounded position goals are generated, then independently replayed and loaded.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json,time
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT,R
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor
from research_quasistatic_wrench_path_20261105 import allocate_native,hybrid_increment
from offline_fractional_goal_accumulator_20261105 import FractionalGoals
from hand_motor_mapping import ticks_to_sim
from canonical_grip_dynamics_20261105 import run
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg
from checkpoint_quarter_review_20261106 import quarter_reasons
from evaluate_canonical_grip_reaction_20261105 import evaluate

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--target-N',type=float,choices=(.8,1.2),required=True);a=ap.parse_args();parent=ROOT/'continuous_handoff/review.json';previous=json.loads(parent.read_bytes());assert previous['quarter_and_handoff_passed'];source=parent.parent/'end_state.npz';model=R/previous['source_model'];assert sha(model)==previous['source_model_sha256'];z=np.load(source);m=mujoco.MjModel.from_binary_path(str(model));d=restore(m,z);hand,_,_=inspect_for(m);cube=m.geom('cube').id;body=m.body('cube').id;C0=d.qpos[-7:-4].copy();B0=d.xmat[body].reshape(3,3).copy();lo,hi,nlo,nhi=native(m);start=motor(d.ctrl);planner=FractionalGoals(start,np.minimum(lo+8,start),np.maximum(hi-8,start));ref={name:a.target_N for name in ('if_tip','th_tip')};frames={};rows=[];out=ROOT/'handoff_preload'/f'normal{a.target_N:g}';out.mkdir(parents=True,exist_ok=True)
    for name in ref:
        g=m.geom(name).id;contacts=[c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,g}]
        if contacts:
            c=min(contacts,key=lambda c:c.dist);normal=c.frame[:3]*(1 if int(c.geom1)==cube else -1);frames[name]=(d.geom_xmat[g].reshape(3,3).T@normal,B0.T@normal)
    generated_stop=None
    for index in range(120):
        C=d.qpos[-7:-4].copy();B=d.xmat[body].reshape(3,3);velocity=np.zeros(6);mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,body,velocity,0);F=-m.body_mass[body]*m.opt.gravity+30*(C0-C)-1.5*velocity[3:];rot=__import__('scipy').spatial.transform.Rotation.from_matrix(B0@B.T).as_rotvec();T=.04*rot-.002*velocity[:3];proposal,allocation=allocate_native(m,d,cube,C,F,T,ref,hand,True,(nlo,nhi),('if_tip','th_tip'),normal_reference_weight=2.)
        if proposal is None:generated_stop=allocation['reason'];break
        goals=hybrid_increment(m,d,allocation,planner,C,C0,B,B0,world_up=z['up'],maximum_tracking_rad=.12,contact_normal_frames=frames);d.ctrl[:]=ticks_to_sim(goals);rows.append(goals.copy())
        for sub in range(75):mujoco.mj_step(m,d)
        if np.linalg.norm(d.qpos[-7:-4]-C0)>.01 or orientation_error_deg(d.xmat[body].reshape(3,3),B0)>10:generated_stop='proposal_cube_pose_diverged';break
    # The generator above is proposal-only. Every1ms collision/range/current-
    # proxy/tracking/retention check is performed in this fresh replay.
    rows=np.vstack([rows,np.tile(rows[-1],(134,1))]);np.savez_compressed(out/'candidate.npz',start=start,goals=rows);v=run(m,z,rows,1,out/'end_state.npz',out/'trajectory.npz',True,5,('if_tip','th_tip'));end=restore(m,np.load(out/'end_state.npz'));target=upright_yaw_target(z['initial_cube_rotation'],z['up'],90);why=quarter_reasons(v,C0,True,orientation_error_deg(end.xmat[body].reshape(3,3),target));loads=[]
    if not why:
        steady=dict(np.load(out/'end_state.npz'));steady['required_lower_contacts']=np.asarray(['if_tip','th_tip'])
        for torque in (0.,-.003,.003,-.01,.01):
            load=evaluate(m,steady,torque,out/f'load{torque:+g}.npz');loads.append(load);print(json.dumps(dict(target_N=a.target_N,torque=torque,passed=load['load_filter_passed'],yaw_drift=load['max_yaw_change_deg'],reasons=load['reasons'])),flush=True)
    report=dict(status='final_grip_position_preload_refinement',target_normal_N=a.target_N,source_parent_review_sha256=sha(parent),source_state=source.relative_to(R).as_posix(),source_state_sha256=sha(source),source_model_sha256=sha(model),generated_stop=generated_stop,candidate_sha256=sha(out/'candidate.npz'),dynamic=v,quarter_and_handoff_passed=not why,reasons=why,hypothetical_load_cases=loads,all_nominal_hypothetical_loads_passed=bool(loads) and all(c['load_filter_passed'] for c in loads),whole_initial_path_not_replayed_with_this_refinement=True,actual_current_limits_PID_and_native_ranges_unchanged=True,devices_opened=False,hardware_execution_allowed=False);(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(target_N=a.target_N,safe=v['safety_passed'],handoff=not why,yaw=v['final_yaw_from_placement_deg'],tilt=v['final_up_tilt_deg'],gap=v['settled_tail']['minimum_separating_plane_gap_1ms_mm'],contacts=v['final_contact_N'],reasons=why,loads_passed=sum(c['load_filter_passed'] for c in loads))),flush=True)
if __name__=='__main__':main()
