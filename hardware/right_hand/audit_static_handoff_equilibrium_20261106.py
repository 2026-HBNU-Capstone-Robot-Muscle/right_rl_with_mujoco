"""Check complete native gravity/reaction wrenches at existing pinned poses.

This is a static proposal audit, never a motion or force injected into a cube.
Geometry, native actuator authority and90deg pose are checked independently.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import hashlib,json
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from dynamic_common_grip_search_20261105 import ROOT,R
from research_rotation_body_hold_20261104 import restore,inspect_for,native,angle,motor
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from research_quasistatic_wrench_path_20261105 import allocate_native
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    model=ROOT/'torque_cap_sensitivity/cap0.15/model.mjb';source=ROOT/'near_quarter_full_replay/end_state.npz';z=np.load(source);m=mujoco.MjModel.from_binary_path(str(model));hand,support,check=inspect_for(m);geometry=precompute(m,hand);cube=m.geom('cube').id;lo,hi,nlo,nhi=native(m);out=ROOT/'static_handoff_equilibrium';out.mkdir(exist_ok=True);cases=[]
    for label,path,roles in (('if_th_upright',ROOT/'lower_countergrip_station/seed1/static_proposal.npz',('if_tip','th_tip')),('mf_th_shifted',ROOT/'postturn_regrasp_refined/middle_zneg/static_proposal.npz',('mf_tip','th_tip')),('if_th_balanced_centered',ROOT/'lower_countergrip_station_force2_span12_equilibrium/seed4/static_proposal.npz',('if_tip','th_tip'))):
        p=np.load(path);d=restore(m,z);d.qpos[:16]=p['actual_joint_proposal'];d.qpos[-7:-4]=p['cube_center'];B=p['cube_rotation'];q=Rotation.from_matrix(B).as_quat();d.qpos[-4:]=np.r_[q[3],q[:3]];d.qvel[:]=0;d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d);C=d.qpos[-7:-4].copy();clear=min(v for name,v in gaps(m,d,C,B,geometry,True));su,sp=check(d);pen=max((max(0.,-float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2))),default=0.);target=upright_yaw_target(z['initial_cube_rotation'],z['up'],90);orientation=orientation_error_deg(B,target);native_actual=float(max(0,max(nlo-d.qpos[:16]),max(d.qpos[:16]-nhi)));rows=[];folder=out/label;folder.mkdir(exist_ok=True)
        for torque in (0.,-.003,.003,-.01,.01):
            ctrl,allocation=allocate_native(m,d,cube,C,-m.body_mass[m.body('cube').id]*m.opt.gravity,z['up']*torque,{role:2. for role in roles},hand,True,(nlo,nhi),roles,normal_reference_weight=2.);feasible=False;goals=None;residual=None
            if ctrl is not None:
                residual=np.asarray(allocation['desired_wrench_tracking_residual']);goals=motor(ctrl);feasible=bool(max(abs(residual[:3]))<.005 and max(abs(residual[3:]))<.0002 and np.all(goals>=lo) and np.all(goals<=hi) and all(allocation['normal_N'].get(role,0)>.2 for role in roles));np.savez_compressed(folder/f'gravity_torque{torque:+g}.npz',actual_joint_proposal=d.qpos[:16],cube_center=C,cube_rotation=B,force_balanced_control=ctrl,goal_ticks=goals,required_lower_contacts=np.asarray(roles))
            rows.append(dict(reaction_torque_Nm=torque,static_native_wrench_feasible=feasible,residual=None if residual is None else residual.tolist(),allocation={k:v for k,v in allocation.items() if k!='contact_tasks'}))
        geometry_ok=bool(clear>.3 and max(sp.values(),default=0.)<=1e-6 and max(su.values(),default=0.)<=.15 and pen<=1 and native_actual<=1e-6);pose_ok=bool(abs((angle(B,z['initial_cube_rotation'],z['up'])-90+180)%360-180)<5 and np.degrees(np.arccos(np.clip((-B[:,1])@z['up'],-1,1)))<5 and orientation<5);row=dict(label=label,source_proposal=path.relative_to(R).as_posix(),source_proposal_sha256=sha(path),geometry_passed=geometry_ok,target90_pose_passed=pose_ok,upper_gap_mm=clear,orientation_error_deg=orientation,source_yaw_deg=angle(B,z['initial_cube_rotation'],z['up']),self_mm=max(sp.values(),default=0.),support_mm=max(su.values(),default=0.),cube_mm=pen,required_lower_contacts=list(roles),wrench_cases=rows,static_handoff_all_cases_passed=geometry_ok and pose_ok and all(r['static_native_wrench_feasible'] for r in rows));cases.append(row);(out/'review.json').write_text(json.dumps(dict(status='native_static_handoff_gravity_and_load_audit',source_model_sha256=sha(model),source_reference_state_sha256=sha(source),cases=cases,not_free_cube_motion_evidence=True,no_external_cube_forces_written=True,devices_opened=False,hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8');print(json.dumps(row),flush=True)
if __name__=='__main__':main()
