"""Compare measured hand motion and measured goal motion in a free-cube model.

Prescribed hand positions require effectively infinite hand inertia. This
ISOLATED diagnostic changes hand armature and overwrites hand joint states only.
It cannot establish actuator force, live safety or permission to run a path.
The cube is never pinned, reset or teleported after the diagnostic starts.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json,time
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT,R
from research_rotation_body_hold_20261104 import restore,inspect_for,native,angle
from cube_contact_material_policy_20261029 import apply_material
from hand_motor_mapping import ticks_to_sim

PHASES={'stage_5_loaded_grasp','stage_6_grasp_hold','stage_7_horizontal_yaw','stage_8_horizontal_hold'}

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--friction',type=float,choices=(.2,.3,.5),default=.3);a=ap.parse_args()
    source=R/'forward_hold_observe_20261105_040030_646453/result.json';result=json.loads(source.read_bytes());rows=[s for s in result['samples'] if s['phase'] in PHASES]
    if len(rows)<1000 or not result['all_torque_off_verified']:raise ValueError('incomplete historical source')
    times=np.array([r['t'] for r in rows]);times-=times[0]
    if np.any(np.diff(times)<=0) or np.max(np.diff(times))>.25:raise ValueError('recorded chronology/gap exceeds diagnostic scope')
    out=ROOT/f'model_discrepancy/recorded_joints_f{a.friction:g}';out.mkdir(parents=True,exist_ok=True)
    folder=ROOT/'full_placement_v2/front-10/g1_f0.3';cases=[]
    for field in ('goal','actual'):
        began=time.perf_counter();m=mujoco.MjModel.from_binary_path(str(folder/'model.mjb'));z=np.load(folder/'initial_state.npz');d=restore(m,z);hand,support,check=inspect_for(m);apply_material(m,a.friction,hand,m.geom('cube').id);_,_,nlo,nhi=native(m)
        raw=np.array([r[field] for r in rows],int);positions=ticks_to_sim(raw);step=m.opt.timestep
        grid=np.arange(0,times[-1]+step/2,step);poses=np.column_stack([np.interp(grid,times,positions[:,j]) for j in range(16)]);vel=np.gradient(poses,step,axis=0)
        m.dof_armature[:16]=1e6
        d.qpos[:16]=poses[0];d.qvel[:16]=vel[0];d.ctrl[:]=poses[0];mujoco.mj_forward(m,d)
        body=m.body('cube').id;cube=m.geom('cube').id;C0=d.qpos[-7:-4].copy();B0=d.xmat[body].reshape(3,3).copy();up=z['up'];worst=dict(self_mm=0.,support_mm=0.,cube_mm=0.,measured_range_rad=0.);stop=None;longest=gap=0.;external=False;trace=[];snapshots=[];records=[]
        for i,t in enumerate(grid):
            d.qpos[:16]=poses[i];d.qvel[:16]=vel[i];d.ctrl[:]=poses[i];mujoco.mj_forward(m,d)
            su,sp=check(d);worst['self_mm']=max(worst['self_mm'],max(sp.values(),default=0.));worst['support_mm']=max(worst['support_mm'],max(su.values(),default=0.));worst['measured_range_rad']=max(worst['measured_range_rad'],float(max(0,max(nlo-d.qpos[:16]),max(d.qpos[:16]-nhi))));touch=False;contacts={}
            for ci,c in enumerate(d.contact):
                a0,b0=int(c.geom1),int(c.geom2)
                if cube not in (a0,b0):continue
                other=b0 if a0==cube else a0
                if int(m.geom_bodyid[other]) in hand:
                    touch|=c.dist<.0004;worst['cube_mm']=max(worst['cube_mm'],max(0,-float(c.dist)*1000));ff=np.zeros(6);mujoco.mj_contactForce(m,d,ci,ff);contacts[m.geom(other).name]=contacts.get(m.geom(other).name,0.)+float(ff[0])
                elif c.dist<.0004 and (other in support or int(m.geom_bodyid[other])==0):external=True
            gap=0. if touch else gap+step;longest=max(longest,gap)
            B=d.xmat[body].reshape(3,3);C=d.qpos[-7:-4];yaw=angle(B,B0,up);tilt=float(np.degrees(np.arccos(np.clip((-B[:,1])@up,-1,1))));shift=float(np.linalg.norm(C-C0)*1000)
            if i%750==0 or i==len(grid)-1:
                trace.append(dict(t=float(t),yaw_deg=yaw,up_tilt_deg=tilt,shift_mm=shift,contact_N=contacts));snapshots.append(d.qpos.copy());records.append(float(t))
            if shift>80 or not np.isfinite(d.qpos).all():stop='cube displacement or nonfinite diagnostic limit';break
            mujoco.mj_step(m,d)
        np.savez_compressed(out/(field+'_trajectory.npz'),qpos=snapshots,times=records)
        row=dict(prescribed_signal=field,completed=stop is None,stop=stop,simulated_seconds=float(grid[i]),final_yaw_deg=yaw,final_up_tilt_deg=tilt,final_shift_mm=shift,worst=worst,external_cube_contact=external,maximum_hand_contact_gap_s=longest,trace=trace,elapsed_seconds=time.perf_counter()-began);cases.append(row)
        report=dict(status='prescribed_historical_joint_diagnostic_only',source=str(source.relative_to(R)),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),physical_recorded_rows=len(rows),friction=a.friction,cases=cases,hand_armature_changed_to_1e6=True,hand_joint_states_prescribed=True,cube_never_reset_after_start=True,initial_cube_pose_not_physically_registered=True,not_an_actuator_or_current_or_live_safety_test=True,not_a_candidate_path_validation=True,hardware_execution_allowed=False,devices_opened=False)
        (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in row.items() if k!='trace'}),flush=True)
if __name__=='__main__':main()
