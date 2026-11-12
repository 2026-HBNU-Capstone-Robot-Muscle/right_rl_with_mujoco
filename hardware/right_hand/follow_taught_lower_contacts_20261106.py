"""Offline perfect-state contact following; never a live hardware policy."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,time
from pathlib import Path
import numpy as np,mujoco
from scipy.optimize import lsq_linear
from research_taught_grasp_20261106 import O,sha,save_json,details
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor
from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK,ticks_to_sim
from offline_fractional_goal_accumulator_20261105 import FractionalGoals
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from canonical_grip_dynamics_20261105 import run

def generate(m,z,target_force,clear_margin):
    d=restore(m,z);p=restore(m,z);hand,_,check=inspect_for(m);cache=precompute(m,hand);cube=m.geom('cube').id;lo,hi,_,_=native(m);start=motor(d.ctrl);active=np.r_[np.arange(4,8),np.arange(12,16)];ids=np.asarray(SIM_TO_MOTOR_IDS)-1
    planner=FractionalGoals(start,np.minimum(lo+8,start),np.maximum(hi-8,start));rows=[];trace=[];frames={};stop=None;C0=d.qpos[-7:-4].copy();began=time.perf_counter()
    def collision_clearances(data):
        return gaps(m,data,data.geom_xpos[cube],data.geom_xmat[cube].reshape(3,3),cache,True)
    for index in range(400):
        C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3);up=-B[:,1];matrix=[];rhs=[];forces={}
        for name in ('mf_tip','th_tip'):
            g=m.geom(name).id;points=np.zeros(6);dist=mujoco.mj_geomDistance(m,d,cube,g,.15,points);contacts=[(ci,c) for ci,c in enumerate(d.contact) if {int(c.geom1),int(c.geom2)}=={cube,g}];force=0.
            if contacts:
                ci,c=min(contacts,key=lambda x:x[1].dist);ff=np.zeros(6);mujoco.mj_contactForce(m,d,ci,ff);force=float(ff[0]);n=c.frame[:3]*(1 if int(c.geom1)==cube else -1);point=c.pos.copy();frames[name]=n.copy()
            else:
                point=points[3:].copy();n=(points[3:]-points[:3])/max(abs(dist),1e-12)
                if not np.isfinite(n).all() or np.linalg.norm(n)<.9:n=frames.get(name,np.array([1.,0.,0.]))
            forces[name]=force;jp=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jp,None,point,int(m.geom_bodyid[g]));J=jp[:,active];row=n@J
            closing=min(.00010,max(0.,float(dist))*.18)+float(np.clip((target_force-force)*np.sum(row**2/m.actuator_gainprm[active,0])*.08,-.00006,.00006))
            matrix.append(row[None,:]*10000);rhs.append(np.array([-closing])*10000)
            height=float((point-C)@up);slide=float(np.clip((.003-height)*.08,-.00006,.00006));matrix.append((up@J)[None,:]*1500);rhs.append(np.array([slide])*1500)
        clear=collision_clearances(d);critical=sorted(clear,key=lambda x:x[1])[:8];gradient=np.zeros((len(critical),len(active)))
        for j,qi in enumerate(active):
            p.qpos[:]=d.qpos;p.qpos[qi]+=.0001;p.qvel[:]=0;mujoco.mj_forward(m,p);shifted=dict(collision_clearances(p))
            gradient[:,j]=[(shifted[n]-v)/.0001 for n,v in critical]
        for (name,value),grad in zip(critical,gradient):
            if value<clear_margin:
                matrix.append(grad[None,:]*4);rhs.append(np.array([min(.08,(clear_margin-value)*.1)])*4)
        matrix.append(np.eye(len(active))*.02);rhs.append(np.zeros(len(active)));lower,upper=planner.increment_bounds();lower=lower[ids[active]]*TICK;upper=upper[ids[active]]*TICK;fixed=upper-lower<1e-12;change=np.zeros(len(active));A=np.vstack(matrix);v=np.concatenate(rhs)
        if np.any(~fixed):change[~fixed]=lsq_linear(A[:,~fixed],v-A[:,fixed]@change[fixed],bounds=(lower[~fixed],upper[~fixed]),tol=1e-9,max_iter=100).x
        inc=np.zeros(16);inc[ids[active]]=change/TICK;goal=planner.advance(inc);rows.append(goal);d.ctrl[:]=ticks_to_sim(goal)
        for _ in range(75):mujoco.mj_step(m,d)
        su,sp=check(d)
        if max(sp.values(),default=0.)>1e-6 or max(su.values(),default=0.)>.15 or max(abs(d.qpos[:16]-d.ctrl))>.18 or np.linalg.norm(d.qpos[-7:-4]-C0)>.015:stop='proposal guard';break
        if index%50==0:
            trace.append(dict(goal=index+1,forces=forces,gap_mm=min(v for _,v in clear)));print(json.dumps(dict(target_N=target_force,clear_margin_mm=clear_margin,proposal=trace[-1])),flush=True)
    return np.vstack([rows,np.tile(rows[-1],(134,1))]),dict(proposal_stop=stop,trace=trace,seconds=time.perf_counter()-began)

def main():
    out=O/'contact_following';out.mkdir(exist_ok=True);source=O/'sliding_contact/shape1_height3/middle_first_end.npz';z=np.load(source);m=mujoco.MjModel.from_binary_path(str(O/'model.mjb'));cases=[]
    for force,clear in ((.3,.8),(.5,1.5)):
        folder=out/f'normal{force:g}_gap{clear:g}';folder.mkdir(exist_ok=True);rows,gen=generate(m,z,force,clear);np.savez_compressed(folder/'candidate.npz',start=motor(z['ctrl']),goals=rows);result=run(m,z,rows,1,folder/'end.npz',folder/'trajectory.npz',True);row=dict(target_normal_N=force,clearance_target_mm=clear,generation=gen,dynamic=result,endpoint=details(m,np.load(folder/'end.npz')));cases.append(row);save_json(folder/'review.json',row)
        save_json(out/'review.json',dict(status='perfect_simulated_contact_following_research',cases=cases,source_state_sha256=sha(source),model_sha256=sha(O/'model.mjb'),source_script_sha256=sha(Path(__file__)),real_camera_contact_feedback_not_implemented=True,real_current_PID_limits_unchanged=True,devices_opened=False,hardware_execution_allowed=False,physical_success=False))
        print(json.dumps(dict(target_N=force,safe=result['safety_passed'],passed=result['whole_grip_task_passed'],tail=result['settled_tail'],contacts=result['final_contact_N'])),flush=True)
if __name__=='__main__':main()
