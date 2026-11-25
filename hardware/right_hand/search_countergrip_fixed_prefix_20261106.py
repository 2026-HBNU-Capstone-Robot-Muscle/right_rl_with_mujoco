"""Inspect recorded transient poses, then independently replay fixed endpoints.

Only original goal prefixes plus134 repeated goals are tried. Geometry-only
samples are not successes; pressure, all1ms guards and the unchanged90 filter
must pass a fresh free-cube replay. No cube resets during the replay.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import hashlib,json,time
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT,R
from research_rotation_body_hold_20261104 import restore,inspect_for,angle
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from canonical_grip_dynamics_20261105 import run
from checkpoint_quarter_review_20261106 import quarter_reasons

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    tags=('seed0_source1_lift0_normal2.5_n900_resume_lower_staged_freebase','seed0_source1_lift0_normal1.5_n900_finish_freebase_tol3_local_station')
    model=ROOT/'torque_cap_sensitivity/cap0.15/model.mjb';m=mujoco.MjModel.from_binary_path(str(model));hand,_,check=inspect_for(m);geometry=precompute(m,hand);cube=m.geom('cube').id;out=ROOT/'countergrip_fixed_prefix';out.mkdir(exist_ok=True);results=[];began=time.perf_counter()
    for tag in tags:
        folder=ROOT/'countergrip_level'/tag;r=json.loads((folder/'review.json').read_bytes());source=R/r['source_state'];z=np.load(source);d=restore(m,z);targetB=upright_yaw_target(z['initial_cube_rotation'],z['up'],90);targetC=np.asarray(r['planned_new_common_center_world_m']);rows=np.load(folder/'suffix.npz')['goals'];poses=np.load(folder/'trajectory.npz')['qpos'];assert len(poses)==(len(rows)+1)//2 and len(rows)%2==0
        samples=[]
        for j,q in enumerate(poses):
            sample=j*2+1;d.qpos[:]=q;d.qvel[:]=0;mujoco.mj_forward(m,d);C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3);yaw=angle(B,z['initial_cube_rotation'],z['up']);tilt=float(np.degrees(np.arccos(np.clip((-B[:,1])@z['up'],-1,1))));err=orientation_error_deg(B,targetB);center=float(np.linalg.norm(C-targetC)*1000);gap=min(v for name,v in gaps(m,d,C,B,geometry,True));possible=bool(abs((yaw-90+180)%360-180)<5 and tilt<5 and err<5 and center<2 and gap>.3);samples.append(dict(sample=sample,yaw_deg=yaw,tilt_deg=tilt,orientation_error_deg=err,center_error_mm=center,upper_gap_mm=gap,static_possible=possible))
        candidates=[s for s in samples if s['static_possible']];ranked=sorted(candidates,key=lambda s:min(s['upper_gap_mm']-.3,2-s['center_error_mm'],5-s['orientation_error_deg']),reverse=True);selected=[]
        for s in ranked:
            if all(abs(s['sample']-c['sample'])>=12 for c in selected):selected.append(s)
            if len(selected)>=6:break
        caseout=out/tag;caseout.mkdir(exist_ok=True);replays=[]
        for s in selected:
            k=s['sample'];goals=np.vstack([rows[:k],np.tile(rows[k-1],(134,1))]);np.savez_compressed(caseout/f'prefix{k}.npz',start=np.load(folder/'suffix.npz')['start'],goals=goals);v=run(m,z,goals,1,caseout/f'prefix{k}_end.npz',None,True,5,('if_tip','th_tip'));final=restore(m,np.load(caseout/f'prefix{k}_end.npz'));why=quarter_reasons(v,targetC,True,orientation_error_deg(final.geom_xmat[cube].reshape(3,3),targetB));replays.append(dict(prefix_goals=k,dynamic=v,reasons=why,checkpoint90_and_grip_passed=not why));print(json.dumps(dict(source=tag,prefix=k,safety=v['safety_passed'],yaw=v['final_yaw_from_placement_deg'],passed=not why,reasons=why)),flush=True)
        row=dict(source_review=(folder/'review.json').relative_to(R).as_posix(),source_review_sha256=sha(folder/'review.json'),source_state=source.relative_to(R).as_posix(),source_state_sha256=sha(source),source_goals_sha256=sha(folder/'suffix.npz'),source_trajectory_sha256=sha(folder/'trajectory.npz'),samples=samples,geometric_candidates=len(candidates),dynamic=replays);results.append(row);report=dict(status='recorded_prefix_geometry_scan_and_fixed_goal_replays',source_model_sha256=sha(model),cases=results,odd_sample_indices_verified_from_source_sampling_rule=True,static_samples_not_motion_trials=True,cube_not_reset_during_replays=True,devices_opened=False,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began);(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(source=tag,geometric_candidates=len(candidates),replays=len(replays),passed=sum(c['checkpoint90_and_grip_passed'] for c in replays))),flush=True)
if __name__=='__main__':main()
