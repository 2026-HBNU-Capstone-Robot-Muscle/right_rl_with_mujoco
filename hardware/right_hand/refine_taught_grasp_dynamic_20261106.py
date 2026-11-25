"""Free-cube goal search; sampled proposals require separate 1ms verification."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,json,time
from pathlib import Path
import numpy as np,mujoco
from mujoco import rollout
from scipy.optimize import differential_evolution
from scipy.spatial.transform import Rotation
from research_taught_grasp_20261106 import O,R,sha,save_json,details
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor,angle
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS,TICK
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from canonical_grip_dynamics_20261105 import run
from dynamic_common_grip_search_20261105 import ramp

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--strict-proposals',action='store_true');args=ap.parse_args()
    out=O/('dynamic_refinement_strict' if args.strict_proposals else 'dynamic_refinement');out.mkdir(exist_ok=True);source=O/'sliding_contact/shape1_height3/middle_first_end.npz';z=np.load(source);m=mujoco.MjModel.from_binary_path(str(O/'model.mjb'));d=restore(m,z);work=mujoco.MjData(m);hand,_,check=inspect_for(m);cache=precompute(m,hand);cube=m.geom('cube').id;body=m.body('cube').id
    start=motor(d.ctrl);active=np.r_[np.arange(4),np.arange(8,12)];lo,hi,_,_=native(m);lower=np.maximum(-96,np.ceil(lo[active])+8-start[active]);upper=np.minimum(96,np.floor(hi[active])-8-start[active]);spec=mujoco.mjtState.mjSTATE_FULLPHYSICS;initial=np.zeros(mujoco.mj_stateSize(m,spec));mujoco.mj_getState(m,d,initial,spec);C0=d.qpos[-7:-4].copy();B0=d.xmat[body].reshape(3,3).copy();ids=np.asarray(SIM_TO_MOTOR_IDS)-1;began=time.perf_counter();count=0;best=float('inf');bestrow=None;memo={}
    def loss(x):
        nonlocal count,best,bestrow
        key=tuple(np.rint(x).astype(int))
        if key in memo:return memo[key]
        goal=start.copy();goal[active]+=np.array(key);raw=ramp(start,goal,40);ctrl=np.repeat((raw[:,ids]-2048)*TICK,75,axis=0)
        states,_=rollout.rollout(m,mujoco.MjData(m),initial,ctrl,initial_warmstart=z['qacc_warmstart']);state=states[0,-1];work.time=state[0];work.qpos[:]=state[1:1+m.nq];work.qvel[:]=state[1+m.nq:1+m.nq+m.nv];work.ctrl[:]=ctrl[-1];mujoco.mj_forward(m,work)
        C=work.geom_xpos[cube];B=work.geom_xmat[cube].reshape(3,3);clearance=gaps(m,work,C,B,cache,True);gap=min(v for _,v in clearance);forces={'mf_tip':0.,'th_tip':0.};hcontact={};depth=0.
        for ci,c in enumerate(work.contact):
            a,b=int(c.geom1),int(c.geom2)
            if cube not in (a,b):continue
            g=b if a==cube else a;depth=max(depth,max(0.,-float(c.dist)*1000));name=m.geom(g).name
            if name in forces:
                ff=np.zeros(6);mujoco.mj_contactForce(m,work,ci,ff);height=-float((c.pos-C)@B[:,1])*1000;hcontact[name]=height
                if height<55/6:forces[name]+=float(ff[0])
        distance=[]
        for name in forces:
            pts=np.zeros(6);v=mujoco.mj_geomDistance(m,work,cube,m.geom(name).id,.15,pts)*1000;h=-float(((pts[:3]+pts[3:])/2-C)@B[:,1])*1000;distance.append(max(0.,v)+max(0.,h-7.))
        su,sp=check(work);rot=float(np.degrees(np.linalg.norm(Rotation.from_matrix(B@B0.T).as_rotvec())));shift=float(np.linalg.norm(C-C0)*1000);tilt=float(np.degrees(np.arccos(np.clip((-B[:,1])@z['up'],-1,1))))
        cost=4*sum(v*v for v in distance)+35*sum(max(0.,.4-v)**2 for v in forces.values())+8*max(0.,1.2-gap)**2+max(0.,rot-2.)**2+max(0.,shift-1.)**2+3*max(0.,tilt-4.)**2+10000*max(sp.values(),default=0.)**2+500*max(0.,depth-.8)**2+.00003*sum(np.array(key)**2)
        if args.strict_proposals and (max(sp.values(),default=0.)>1e-6 or max(su.values(),default=0.)>.15):cost+=1e6
        if not np.isfinite(cost):cost=1e12
        count+=1;memo[key]=float(cost)
        if cost<best:
            best=cost;bestrow=dict(evaluation=count,cost=cost,offset=list(key),goal=goal.tolist(),gap_mm=gap,forces_N=forces,tip_distance_mm=distance,contact_heights_mm=hcontact,shift_mm=shift,rotation_change_deg=rot,tilt_deg=tilt,self_mm=max(sp.values(),default=0.),proposals_are_not_full_safety_reviews=True)
            bestrow['offset']=[int(v) for v in key];save_json(out/'search_best.json',bestrow);print(json.dumps(bestrow),flush=True)
        return float(cost)
    loss(np.zeros(8));bounds=list(zip(lower,upper));fit=differential_evolution(loss,bounds,maxiter=9 if args.strict_proposals else 14,popsize=4 if args.strict_proposals else 5,seed=61107 if args.strict_proposals else 61106,polish=False,updating='immediate',tol=.001)
    goal=np.array(bestrow['goal']);rows=ramp(start,goal,134);np.savez_compressed(out/'candidate.npz',start=start,goals=rows)
    dynamic=run(m,z,rows,1,out/'end.npz',out/'trajectory.npz',True)
    report=dict(status='dynamic_taught_grasp_refinement_offline',source_state=source.relative_to(R).as_posix(),source_sha256=sha(source),model_sha256=sha(O/'model.mjb'),search_evaluations=count,search_best=bestrow,dynamic=dynamic,endpoint=details(m,np.load(out/'end.npz')),source_script_sha256=sha(Path(__file__)),all_other_finger_goals_unchanged=True,coarse_search_does_not_certify_path=True,final_review_every_1ms=True,devices_opened=False,hardware_execution_allowed=False,physical_success=False,elapsed_seconds=time.perf_counter()-began);save_json(out/'review.json',report)
    print(json.dumps(dict(final_safety=dynamic['safety_passed'],final_task=dynamic['whole_grip_task_passed'],elapsed=report['elapsed_seconds'],evaluations=count)),flush=True)
if __name__=='__main__':main()
