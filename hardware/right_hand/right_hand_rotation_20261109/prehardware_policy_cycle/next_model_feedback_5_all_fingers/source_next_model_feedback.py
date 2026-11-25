"""Offline free-physics feedback teacher for the small next-turn subproblem.

Prediction copies are only planning branches. One persistent actual model state
executes every selected goal and is never replaced by a pinned/reference cube.
This is a model-based teacher probe, not a learned SAC success or hardware path.
"""
import argparse
import time
import json
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from scipy.optimize import lsq_linear
from native_cycle import OUT,assets,restore,Guard,motor,ticks_to_sim,BOUND,save_json
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg
from upper_layer_cylinder_clearance_20261105 import precompute,gaps


def main():
    p=argparse.ArgumentParser();p.add_argument('--turn',type=float,default=5.);p.add_argument('--steps',type=int,default=220)
    p.add_argument('--tag',default='all_fingers');a=p.parse_args()
    m,initial,_,_,provenance=assets('active')
    source=OUT/'sac_current_contract_v4_20k/export_000000010256/evaluation_active_certified/end_state.npz'
    z=dict(np.load(source,allow_pickle=False));d=restore(m,z);cube=m.geom('cube').id
    start_B=d.geom_xmat[cube].reshape(3,3).copy();C0=d.qpos[-7:-4].copy()
    target=upright_yaw_target(start_B,z['up'],a.turn);active=np.arange(16)
    checker=Guard(m);cache=precompute(m,checker.hand);rows=[];trace=[];remainder=np.zeros(len(active))
    def evidence(data):
        B=data.geom_xmat[cube].reshape(3,3);C=data.qpos[-7:-4]
        rot=Rotation.from_matrix(target@B.T).as_rotvec()*180/np.pi
        force={}
        for ci,c in enumerate(data.contact):
            x,y=int(c.geom1),int(c.geom2)
            if cube not in (x,y):continue
            other=y if x==cube else x;name=m.geom(other).name
            if name in ('if_tip','th_tip') and -(c.pos-C)@B[:,1]*1000<55/6:
                f=np.zeros(6);mujoco.mj_contactForce(m,data,ci,f);force[name]=force.get(name,0.)+float(f[0])
        gap=min(v for _,v in gaps(m,data,data.geom_xpos[cube],B,cache,True))
        error=orientation_error_deg(B,target);shift=float(np.linalg.norm(C-C0)*1000)
        force_penalty=np.array([5*max(0.,.3-force.get(name,0.)) for name in ('if_tip','th_tip')])
        gap_penalty=5*max(0.,.8-gap)
        residual=np.r_[rot,(C-C0)*1000*.5,gap_penalty,force_penalty]
        score=error+.3*shift+float(force_penalty.sum())+gap_penalty
        return residual,score,{'error_deg':error,'shift_mm':shift,'upper_gap_mm':gap,'lower_forces':force}
    def predict(goal):
        state={key:getattr(d,key).copy() for key in BOUND};state['time']=d.time
        trial=restore(m,state);trial.ctrl[:]=ticks_to_sim(goal);guard=Guard(m)
        guard.gap=checker.gap
        if np.any(goal<guard.lo) or np.any(goal>guard.hi):return None
        for _ in range(75):
            mujoco.mj_step(m,trial)
            if guard.check(trial):return None
        return trial,evidence(trial)
    began=time.perf_counter();stop=None;held=0;best_score=float('inf');stale=0
    for step in range(a.steps):
        previous=motor(d.ctrl);zero=predict(previous)
        if zero is None:stop='current_hold_not_predicted_safe';break
        residual0=zero[1][0];candidates=[(previous,zero)]
        J=np.zeros((len(residual0),len(active)))
        from hand_motor_mapping import SIM_TO_MOTOR_IDS
        mids=np.array(SIM_TO_MOTOR_IDS)-1
        for col,joint in enumerate(active):
            ends=[]
            for sign in (-1,1):
                goal=previous.copy();goal[mids[joint]]+=sign*2;trial=predict(goal)
                if trial is not None:candidates.append((goal,trial));ends.append(trial[1][0])
                else:ends.append(None)
            if all(v is not None for v in ends):J[:,col]=(ends[1]-ends[0])/4.
        A=np.vstack([J,.02*np.eye(len(active))]);b=np.r_[-.35*residual0,np.zeros(len(active))]
        fit=lsq_linear(A,b,bounds=(-2,2),max_iter=100)
        intended=fit.x+remainder;delta=np.clip(np.rint(intended).astype(int),-2,2);remainder=np.clip(intended-delta,-.4999999,.4999999)
        for scale in (1.,.5):
            goal=previous.copy();goal[mids[active]]+=np.rint(delta*scale).astype(int);trial=predict(goal)
            if trial is not None:candidates.append((goal,trial))
        goal,prediction=min(candidates,key=lambda item:item[1][1][1])
        d.ctrl[:]=ticks_to_sim(goal);rows.append(goal.copy())
        for _ in range(75):
            mujoco.mj_step(m,d)
            if checker.check(d):stop='original_dynamic_guard';break
        predicted_error=float(np.max(np.abs(d.qpos-prediction[0].qpos)))
        _,score,actual=evidence(d)
        actual.update(step=step+1,prediction_qpos_error=predicted_error);trace.append(actual)
        if predicted_error>1e-8:stop='prediction_state_alignment';break
        lower=all(actual['lower_forces'].get(n,0.)>.2 for n in ('if_tip','th_tip'))
        good=actual['error_deg']<1.5 and actual['shift_mm']<2 and actual['upper_gap_mm']>.3 and lower
        held=held+1 if good else 0
        stale=stale+1 if score>=best_score-.001 else 0;best_score=min(best_score,score)
        if (step+1)%20==0:print(json.dumps(actual),flush=True)
        if stop or held>=54:break
        if stale>=30:stop='local_model_search_stalled';break
    folder=OUT/f'next_model_feedback_{a.turn:g}_{a.tag}';folder.mkdir(exist_ok=True)
    from pathlib import Path
    (folder/'source_next_model_feedback.py').write_bytes(Path(__file__).read_bytes())
    np.savez_compressed(folder/'research_goals.npz',start=motor(z['ctrl']),goals=rows)
    np.savez_compressed(folder/'end_state.npz',**{key:getattr(d,key) for key in BOUND},time=d.time,up=z['up'],initial_cube_rotation=z['initial_cube_rotation'],initial_cube_center=z['initial_cube_center'])
    report={'status':'offline_model_feedback_teacher_probe','curriculum_turn_deg':a.turn,'steps':len(rows),'stop':stop,
            'success':held>=54,'final':trace[-1] if trace else None,'trace':trace,'worst':checker.worst,
            'model_provenance':provenance,'actual_state_never_reset_or_welded_during_suffix':True,
            'predicted_branches_not_physical_trials':True,'this_is_not_SAC_or_second90_success':True,
            'devices_opened':0,'hardware_execution_allowed':False,'elapsed_seconds':time.perf_counter()-began}
    save_json(folder/'review.json',report);print(json.dumps({key:value for key,value in report.items() if key not in ('trace','model_provenance')}),flush=True)


if __name__=='__main__':main()
