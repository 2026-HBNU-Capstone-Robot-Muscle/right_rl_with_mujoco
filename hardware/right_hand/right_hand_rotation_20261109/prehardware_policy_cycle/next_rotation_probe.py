"""Next-turn geometry and free-cube continuation from an existing lower grip.

Pinned poses are used only to propose IK. The dynamic test starts from the saved
grip once and never resets/welds the cube. A failed fixed-contact proposal does
not prove that a regrasp or learned manipulation is impossible.
"""
import argparse
import time
import numpy as np
import mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from native_cycle import OUT,assets,restore,Guard,motor,ticks_to_sim,BOUND,save_json,angle,TICK,SIM_TO_MOTOR_IDS
from inspect_endpoints import inspect
from cube_task_frame_20261105 import orientation_error_deg


def main():
    p=argparse.ArgumentParser();p.add_argument('--turn',type=float,default=90.);a=p.parse_args()
    if not 0<abs(a.turn)<=90:raise ValueError('Bounded next-turn test requires 0<abs(turn)<=90')
    model,initial,_,_,provenance=assets('archived')
    source=OUT/'baseline_archived_zero/end_state.npz';z=dict(np.load(source,allow_pickle=False))
    d=restore(model,z);proposal=restore(model,z);guard=Guard(model)
    cube=model.geom('cube').id;B=d.geom_xmat[cube].reshape(3,3).copy();C=d.geom_xpos[cube].copy()
    active=np.array([0,1,2,3,12,13,14,15]);lag=d.ctrl-d.qpos[:16]
    contacts=[]
    for name in ('if_tip','th_tip'):
        gid=model.geom(name).id
        contact=min((c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,gid}),key=lambda c:c.dist)
        G=d.geom_xmat[gid].reshape(3,3)
        n=contact.frame[:3]*(1 if int(contact.geom1)==cube else -1)
        contacts.append((gid,G.T@(contact.pos-d.geom_xpos[gid]),G.T@n,B.T@(contact.pos-C),B.T@n))
    current=d.qpos[:16].copy();previous=motor(d.ctrl);rows=[];geometry=[];failure=None
    sid=np.array(SIM_TO_MOTOR_IDS)-1
    lower=(np.ceil(guard.lo[sid])+4-2048)*TICK-lag
    upper=(np.floor(guard.hi[sid])-4-2048)*TICK-lag
    began=time.perf_counter();nstep=int(np.ceil(abs(a.turn)))
    for k in range(1,nstep+1):
        theta=a.turn*k/nstep;target=Rotation.from_rotvec(z['up']*np.radians(theta)).as_matrix()@B
        quat=Rotation.from_matrix(target).as_quat()[[3,0,1,2]];seed=current[active].copy()
        def residual(x):
            proposal.qpos[:]=z['qpos'];proposal.qpos[:16]=current;proposal.qpos[active]=x
            proposal.qpos[-7:-4]=C;proposal.qpos[-4:]=quat;proposal.ctrl[:]=proposal.qpos[:16]+lag
            mujoco.mj_forward(model,proposal);e=[];ne=[]
            for gid,local,local_n,clocal,cn in contacts:
                G=proposal.geom_xmat[gid].reshape(3,3)
                e.extend((proposal.geom_xpos[gid]+G@local-(C+target@clocal))*1000)
                ne.extend((G@local_n-target@cn)*6)
            su,sp=guard.contacts(proposal)
            pen=0.
            for c in proposal.contact:
                if cube in (int(c.geom1),int(c.geom2)):pen=max(pen,-float(c.dist)*1000)
            return np.r_[e,ne,(x-seed)*.2,max(sp.values(),default=0.)*500,
                         max(su.values(),default=0.)*500,max(0.,pen-.8)*100]
        lo=np.maximum(lower[active],seed-.12);hi=np.minimum(upper[active],seed+.12)
        fit=least_squares(residual,np.clip(seed,lo,hi),bounds=(lo,hi),max_nfev=80,
                          ftol=1e-8,xtol=1e-8,gtol=1e-8)
        error=residual(fit.x);current[active]=fit.x
        su,sp=guard.contacts(proposal);goal=motor(current+lag)
        tip_error=float(np.max(np.linalg.norm(error[:6].reshape(2,3),axis=1)))
        why=[]
        if tip_error>.8:why.append('fixed_contact_IK_error')
        if max(sp.values(),default=0.)>1e-6:why.append('self_contact')
        if max(su.values(),default=0.)>.15:why.append('support_contact')
        if np.any(goal<guard.lo) or np.any(goal>guard.hi):why.append('native_goal_range')
        geometry.append({'target_increment_deg':theta,'contact_point_error_mm':tip_error,
                         'self_pairs_mm':sp,'support_pairs_mm':su,'reasons':why})
        if why:failure=why;break
        count=max(1,int(np.ceil(np.max(np.abs(goal-previous))/2)))
        rows.extend(np.rint(previous+(goal-previous)*j/count).astype(int) for j in range(1,count+1))
        previous=goal.copy()
    folder=OUT/f'next_fixed_contact_{a.turn:+g}';folder.mkdir(exist_ok=True)
    # Evaluate even a partial IK prefix as a probe; never label it a full90 turn.
    turn_goals=len(rows)
    if rows:rows.extend(previous.copy() for _ in range(134))
    completed=0;dynamic_stop=None;hold_positions=[];hold_rotations=[]
    for index,row in enumerate(rows):
        d.ctrl[:]=ticks_to_sim(row)
        for _ in range(75):
            mujoco.mj_step(model,d);dynamic_stop=guard.check(d)
            if dynamic_stop:break
        completed=index+1
        if index>=turn_goals:
            hold_positions.append(d.qpos[-7:-4].copy());hold_rotations.append(d.xmat[model.body('cube').id].reshape(3,3).copy())
        if dynamic_stop:break
    end=inspect(model,d,z['up'],z['initial_cube_rotation'])
    actual_B=d.xmat[model.body('cube').id].reshape(3,3)
    target=Rotation.from_rotvec(z['up']*np.radians(a.turn)).as_matrix()@B
    np.savez_compressed(folder/'research_goals.npz',start=motor(z['ctrl']),goals=np.asarray(rows,dtype=int))
    np.savez_compressed(folder/'end_state.npz',**{key:getattr(d,key) for key in BOUND},time=d.time,
                        up=z['up'],initial_cube_rotation=z['initial_cube_rotation'],initial_cube_center=z['initial_cube_center'])
    result={'status':'offline_next_turn_fixed_contact_probe','target_increment_deg':a.turn,
            'first_cycle_source':'archived nominal full-path end state; physical robot not used',
            'model_provenance':provenance,'geometry':geometry,'geometry_stop':failure,
            'model_cube_never_reset_or_welded_during_dynamic_suffix':True,
            'dynamic_goals_completed':completed,'dynamic_stop':dynamic_stop,'worst':guard.worst,
            'actual_yaw_increment_deg':angle(actual_B,B,z['up']),
            'next_target_error_deg':orientation_error_deg(actual_B,target),'endpoint':end,
            'hold_center_drift_mm':float(np.max(np.linalg.norm(np.asarray(hold_positions)-hold_positions[0],axis=1))*1000) if hold_positions else None,
            'fixed_contact_failure_is_not_a_proof_against_regrasp':True,
            'task_success_certified':False,'devices_opened':0,'hardware_execution_allowed':False,
            'elapsed_seconds':time.perf_counter()-began}
    save_json(folder/'review.json',result)
    print(__import__('json').dumps({key:value for key,value in result.items() if key not in ('geometry','endpoint','model_provenance')}),flush=True)


if __name__=='__main__':main()
