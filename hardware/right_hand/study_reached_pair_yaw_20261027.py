"""No devices: contact-point IK toward90deg from the reached pair grasp."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,time
from pathlib import Path
import numpy as np,mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS,TICK
from loaded_quarter_reentry_20261025 import limits
from loaded_quarter_model_policy_20261025 import correct_model
from sac_cube_model import set_cube_physics
from screen_loaded_repeat_20261026 import cached_contacts
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261027/reached_pair_yaw_study_v2';OUT.mkdir(exist_ok=True)
def main():
    began=time.perf_counter();z=np.load(ROOT/'right_hand_rotation_20261026/fresh_empty_start_20261026_235258/recovery_plan_v2_ring_first/path.npz');path=z['goals'][z['phases']=='empty_new_grasp'].copy();path[:,7]=np.maximum(path[:,7],2048);opened=z['open_goal'].copy();opened[7]=max(opened[7],2048);base=path[-1];seed=json.loads((ROOT/'right_hand_rotation_20261026/legacy_stage4_seating_v1/seed.json').read_bytes());up=np.array(seed['up_world']);proof=json.loads((ROOT/'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json').read_bytes());nominal=next(r for r in proof['loaded_physics'] if (r['book_front_mm'],r['gain_scale'],r['friction'])==(-10,1,.3));cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,manifest=build(env,OUT/'scene.xml');correct_model(m,np.array(proof['start_ticks']));set_cube_physics(m,.055,.1);m.opt.timestep=.002;m.opt.gravity[:]=-9.81*up;positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004);cube=m.geom('cube').id;m.geom_friction[cube,0]=.3;d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(opened);d.qpos[-7:]=nominal['settled_cube_qpos'];d.ctrl[:]=ticks_to_sim(opened)
    for _ in range(1000):mujoco.mj_step(m,d)
    steps=0
    for sample,q in enumerate(np.vstack([path,np.repeat(base[None,:],40,axis=0)]),1):
        d.ctrl[:]=ticks_to_sim(q);target=round(sample*.075/.002)
        for _ in range(target-steps):mujoco.mj_step(m,d)
        steps=target
    start_state=dict(qpos=d.qpos.copy(),qvel=d.qvel.copy(),ctrl=d.ctrl.copy());np.savez_compressed(OUT/'nominal_grasp_state.npz',**start_state);C=d.qpos[-7:-4].copy();B=d.xmat[m.body('cube').id].reshape(3,3).copy();tips=[m.geom(n+'_tip').id for n in ('th','mf')];centres=np.array([d.geom_xpos[g].copy() for g in tips]);axes=np.array([d.geom_xmat[g].reshape(3,3).copy() for g in tips]);ids=np.array([0,1,2,3,8,9,10,11]);simids=np.array(SIM_TO_MOTOR_IDS)-1;actual=np.empty(16);actual[simids]=d.qpos[:16]/TICK+2048;bias=actual-base;lo,hi=limits();hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};contacts=cached_contacts(m,hand,support);results=[]
    def setup(command,angle,lift):
        D=Rotation.from_rotvec(up*np.radians(angle)).as_matrix();d.qpos[:16]=ticks_to_sim(command+bias);d.qvel[:]=0;d.qpos[-7:-4]=C+up*lift;d.qpos[-4:]=Rotation.from_matrix(D@B).as_quat()[[3,0,1,2]];mujoco.mj_forward(m,d);return D
    def defects():
        su,sp=contacts(d);cube_depth=0.
        for c in d.contact:
            if cube in (int(c.geom1),int(c.geom2)):cube_depth=max(cube_depth,max(0.,-float(c.dist)*1000))
        return max(sp.values(),default=0.),max(su.values(),default=0.),cube_depth
    for lift_mm in (0,4,8):
      for direction in (-1,1):
        previous=base.astype(float);goals=[];log=[];failed=None;targets=[(0,float(h)*.001) for h in range(1,lift_mm+1)]+[(direction*a,lift_mm*.001) for a in range(3,91,3)]
        for angle,lift in targets:
            D=Rotation.from_rotvec(up*np.radians(angle)).as_matrix();positions=C+(centres-C)@D.T+up*lift
            def objective(x):
                command=previous.astype(float).copy();command[ids]=x;setup(command,angle,lift);sp,su,cp=defects();point=np.concatenate([(d.geom_xpos[g]-positions[j])*1000 for j,g in enumerate(tips)]);orientation=np.concatenate([(d.geom_xmat[g].reshape(3,3)-D@axes[j]).ravel()*1.5 for j,g in enumerate(tips)])
                return np.r_[point,orientation,15*sp,15*max(0,su-.1),15*max(0,cp-.9),.001*(x-previous[ids])]
            fit=least_squares(objective,np.clip(previous[ids],lo[ids]+.001,hi[ids]-.001),bounds=(lo[ids]+.001,hi[ids]-.001),diff_step=.00003,max_nfev=90);command=previous.copy();command[ids]=fit.x;command=np.rint(command).astype(int);setup(command,angle,lift);sp,su,cp=defects();error=float(max(np.linalg.norm(d.geom_xpos[g]-positions[j])*1000 for j,g in enumerate(tips)));passed=bool(error<1.5 and sp<=1e-6 and su<=.15 and cp<=1);log.append(dict(angle_deg=angle,lift_mm=lift*1000,max_tip_error_mm=error,self_mm=sp,support_mm=su,cube_mm=cp,passed=passed,goal=command.tolist()))
            if not passed:failed=log[-1];break
            count=max(1,int(np.ceil(max(abs(command-previous))/2)));goals.extend(np.rint(previous+(command-previous)*i/count).astype(int) for i in range(1,count+1));previous=command
        name=f'lift{lift_mm}_dir{direction}';np.savez_compressed(OUT/(name+'.npz'),start=base,goals=np.asarray(goals),bias_ticks=bias);result=dict(name=name,lift_mm=lift_mm,direction=direction,reached_angle_deg=log[-2]['angle_deg'] if failed and len(log)>1 else (0 if failed else log[-1]['angle_deg']),all_endpoints_passed=failed is None,first_failed=failed,goals=len(goals),nominal_seconds=len(goals)*.075,endpoint_log=log);results.append(result);print(json.dumps({k:v for k,v in result.items() if k!='endpoint_log'}),flush=True)
    report=dict(status='same_grasp_yaw_ik_diagnostic',cases=results,source_physical_grasp='new_pair_grasp_20261027_050946_894527/result.json',nominal_model_grasp_qpos=start_state['qpos'].tolist(),nominal_model_cube_qpos=start_state['qpos'][-7:].tolist(),bias_ticks=bias.tolist(),source_contact_tip_centres=centres.tolist(),model_cube_position_hypothesis=True,full_edge_screen_not_done=True,free_cube_yaw_not_checked=True,hardware_execution_allowed=False,real_devices_opened=False,elapsed_seconds=time.perf_counter()-began);(OUT/'ik_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
