"""Offline staged lower regrasp: fit each finger after the previous arrival.

The cube is free throughout every executed command. Static fitting operates on
copies only. Low friction is a design hypothesis, not a material measurement.
"""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
WORK=ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle';sys.path.insert(0,str(WORK))
import mujoco
from scipy.optimize import differential_evolution
from native_cycle import assets,restore,Guard,BOUND,motor,ticks_to_sim,sha,save_json
from whole_hand_support import sample_model
from cube_task_frame_20261105 import aligned_top_face,working_rotation,orientation_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate
from screen_direct_work_grip_20261112 import transition

BASE=ROOT/'right_hand_rotation_20261112/support_first_transfer'
FINGERS={'RF':('rf',np.arange(8,12)),'TH':('th',np.arange(12,16)),'MF':('mf',np.arange(4,8))}
SID=np.array([8,6,7,5,12,10,11,9,16,14,15,13,4,3,2,1])-1

def main():
    p=argparse.ArgumentParser();p.add_argument('--mu',type=float,default=.1);p.add_argument('--tag',default='sequential_low_mu_v1');p.add_argument('--height',type=float,default=-15.);p.add_argument('--from-receiving');p.add_argument('--contact-feedback',action='store_true');a=p.parse_args()
    assert a.tag.replace('_','').isalnum() and 0<=a.mu<=.3
    out=BASE/a.tag;out.mkdir(exist_ok=False);m,initial,_,_,provenance=assets('active_ccd50');m.geom_friction[:,0]=a.mu
    d=restore(m,initial);g=Guard(m);cube=m.geom('cube').id;up=initial['up']/np.linalg.norm(initial['up']);T=2*np.pi/4096
    start=motor(d.ctrl);previous=start.copy();commands=[];phases=[];stages=[];stop=[]
    def save_state(path):np.savez_compressed(path,**{k:getattr(d,k) for k in BOUND},time=d.time)
    def execute(goals,phase):
        nonlocal previous,stop
        for command in goals:
            assert np.max(abs(command-previous))<=2 and np.all(command>=g.lo) and np.all(command<=g.hi)
            d.ctrl[:]=ticks_to_sim(command);previous=command.copy();commands.append(command.copy());phases.append(phase)
            for _ in range(75):
                mujoco.mj_step(m,d);stop=g.check(d)
                if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                if stop:return False
        return True
    acquired=[]
    fractional=np.zeros(16)
    def forces():
        load=sample_model(m,d,g.hand,cube)
        return {f:sum(v['normal_N'] for name,v in load['by_geom'].items() if (m.body(int(m.geom_bodyid[m.geom(name).id])).name or '').startswith(f+'_')) for f in ('rf','mf','th')}
    def pressure_step(key,force):
        f,active=FINGERS[key];tip=m.geom(f+'_tip').id;points=np.zeros(6)
        mujoco.mj_geomDistance(m,d,cube,tip,.5,points);n=points[3:]-points[:3]
        c=next((c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,tip}),None)
        if c is not None:n=c.frame[:3]*(1 if int(c.geom1)==cube else -1)
        elif np.linalg.norm(n)<1e-10:n=d.geom_xpos[tip]-d.geom_xpos[cube]
        n/=max(np.linalg.norm(n),1e-12);J=np.zeros((3,m.nv));mujoco.mj_jac(m,d,J,None,points[3:],int(m.geom_bodyid[tip]));jac=J[:,active]
        distance=float(np.clip((.3-force)*.0001,-.000025,.000025))
        dq=jac.T@np.linalg.solve(jac@jac.T+1e-8*np.eye(3),-n*distance);dt=dq/T
        return dt/max(1,float(np.max(abs(dt)))/2)
    def acquire(target,key):
        nonlocal previous
        f,active=FINGERS[key];ids=SID[active];sustained=0
        for step in range(650):
            measured=forces();command=previous.copy();contacting=measured[f]>.12
            if contacting or key in acquired:
                if key not in acquired:acquired.append(key)
            for held in acquired:
                h,act=FINGERS[held];ix=SID[act];fractional[ix]+=pressure_step(held,measured[h])
                delta=np.clip(np.rint(fractional[ix]),-2,2);command[ix]=np.clip(previous[ix]+delta,np.ceil(g.lo[ix]),np.floor(g.hi[ix])).astype(int)
                fractional[ix]-=command[ix]-previous[ix]
            if key not in acquired:
                command[ids]=previous[ids]+np.clip(target[ids]-previous[ids],-2,2)
                # A model command can wait for contact without exceeding the
                # existing tracking stop; this is not a changed stop threshold.
                actual=motor(d.qpos[:16]);command[ids]=np.clip(command[ids],actual[ids]-64,actual[ids]+64)
                command[ids]=previous[ids]+np.clip(command[ids]-previous[ids],-2,2)
                if np.max(abs(target[ids]-previous[ids]))<=2:
                    fractional[ids]+=pressure_step(key,measured[f]);delta=np.clip(np.rint(fractional[ids]),-2,2)
                    command[ids]=np.clip(previous[ids]+delta,np.ceil(g.lo[ids]),np.floor(g.hi[ids])).astype(int);fractional[ids]-=command[ids]-previous[ids]
            if not execute([command],'contact_acquire_'+f):return False
            now=forces();sustained=sustained+1 if all(now[FINGERS[k][0]]>.1 for k in acquired) and key in acquired else 0
            if sustained>=30:return True
        return False
    # Exact known first turn, upper-middle withdrawal, thumb transfer, paired support.
    prefix=BASE/'corrected_three_finger_continuous_mu0.3/candidate.npz';q=np.load(prefix,allow_pickle=False)['goals'][:2567]
    if a.from_receiving:
        cached=(BASE/a.from_receiving/'receiving_end_state.npz').resolve();cached.relative_to(BASE)
        d=restore(m,dict(np.load(cached,allow_pickle=False)));previous=motor(d.ctrl)
        assert np.array_equal(previous,q[-1])
        commands.extend(q.copy());phases.extend(['cached_prefix']*len(q))
    else:
        for part,(lo,hi) in enumerate(((0,1417),(1417,2123),(2123,2273),(2273,2567))):
            ok=execute(q[lo:hi],f'prefix_{part}')
            print(json.dumps(dict(stage=f'prefix_{part}',done=len(commands),stop=stop)),flush=True)
            if not ok:break
    save_state(out/'receiving_end_state.npz')
    if not stop:
        for k,key in enumerate(('RF','TH','MF')):
            finger,active=FINGERS[key];tip=m.geom(finger+'_tip').id
            saved={n:getattr(d,n).copy() for n in BOUND};saved['time']=float(d.time);copy=restore(m,saved);C=d.geom_xpos[cube].copy();lag=d.ctrl-d.qpos[:16]
            lo=np.maximum(g.qlo,(np.ceil(g.lo[SID])+1-2048)*T-lag);hi=np.minimum(g.qhi,(np.floor(g.hi[SID])-1-2048)*T-lag)
            points=np.zeros(6)
            def measure(x):
                copy.qpos[:]=saved['qpos'];copy.qpos[active]=x;copy.ctrl[:]=copy.qpos[:16]+lag;mujoco.mj_forward(m,copy)
                gap=float(mujoco.mj_geomDistance(m,copy,cube,tip,.5,points))*1000
                height=float((points[:3]-C)@up*1000);support,own=g.contacts(copy)
                pen=max((-float(c.dist)*1000 for c in copy.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
                return gap,height,max(own.values(),default=0.),max(support.values(),default=0.),pen
            def objective(x):
                gap,height,own,support,pen=measure(x)
                return abs(gap+.015)+max(0,height-a.height)*3+own*1500+max(0,support-.1)*500+max(0,pen-.7)*500+.04*np.linalg.norm(x-d.qpos[active])
            rng=np.random.default_rng(3130+k);pop=rng.uniform(lo[active],hi[active],(40,4));pop[0]=np.clip(d.qpos[active],lo[active],hi[active])
            fit=differential_evolution(objective,list(zip(lo[active],hi[active])),init=pop,seed=3130+k,maxiter=100,tol=1e-6,polish=True)
            gap,height,own,support,pen=measure(fit.x);valid=bool(abs(gap)<.15 and height<a.height+.05 and own<=1e-6 and support<=.15 and pen<=1)
            row=dict(finger=finger,static_contact=valid,gap_mm=gap,height_mm=height,self_mm=own,goal_ticks=motor(copy.ctrl).tolist())
            if valid:
                target=previous.copy();target[SID[active]]=motor(copy.ctrl)[SID[active]]
                if a.contact_feedback:ok=acquire(target,key)
                else:
                    moves=transition(previous,target,'all');ok=execute(moves,f'acquire_{finger}')
                    if ok:ok=execute(np.tile(target,(54,1)),f'hold_{finger}')
                row.update(arrival_completed=ok,stop=stop,after_goal=len(commands),loaded=sample_model(m,d,g.hand,cube)['loaded_hand_geoms'])
            else:row['arrival_completed']=False
            stages.append(row);save_state(out/(finger+'_end_state.npz'));save_json(out/'stages.json',dict(rows=stages))
            print(json.dumps(row),flush=True)
            if stop or not valid:break
    # Independent fixed-goal final hold, including every original native guard.
    count=supported=clear=0;loaded={f:0 for f in ('rf','th','mf')};shift=drift=0.;upper_failure=None;minimum=None
    face=aligned_top_face(d.geom_xmat[cube].reshape(3,3),up)['best_candidate_id'];cert=SweepCertificate(m,g.hand,.3)
    C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy()
    if not stop and len(stages)==3 and all(r['arrival_completed'] for r in stages):
        for _ in range(54):
            command=previous.copy();commands.append(command);phases.append('final_three_finger_hold')
            for _ in range(75):
                mujoco.mj_step(m,d);stop=g.check(d)
                if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                load=sample_model(m,d,g.hand,cube);count+=1;supported+=load['supported_sample']
                for f in loaded:loaded[f]+=sum(v['normal_N'] for name,v in load['by_geom'].items() if (m.body(int(m.geom_bodyid[m.geom(name).id])).name or '').startswith(f+'_'))>.1
                B=d.geom_xmat[cube].reshape(3,3);C=d.geom_xpos[cube];shift=max(shift,float(np.linalg.norm(C-C0)*1000));drift=max(drift,orientation_error_deg(B,B0))
                if upper_failure is None:
                    gap,bad=cert.query(d,C,working_rotation(B,face));minimum=gap if minimum is None else min(minimum,gap)
                    if bad or gap<=.3:upper_failure=dict(sample=count,gap_mm=gap,geoms=[m.geom(int(cert.ids[j])).name for j in bad])
                    else:clear+=1
                if stop:break
            if stop:break
    top=aligned_top_face(d.geom_xmat[cube].reshape(3,3),up)
    passed=bool(not stop and count==clear==4050 and supported/count>.8 and all(v/count>.8 for v in loaded.values()) and shift<2 and drift<3 and top['face_id']==face)
    np.savez_compressed(out/'candidate.npz',start=start,goals=commands,phases=phases);save_state(out/'end_state.npz')
    r=dict(status='sequential_lower_contact_offline_preparation',passed=passed,mu_hypothesis=a.mu,contact_height_ceiling_mm=a.height,done=len(commands),stop=stop,
           stages=stages,hold_samples=count,supported_samples=supported,loaded_fractions={f:v/max(1,count) for f,v in loaded.items()},
           upper_certified_samples=clear,upper_first_failure=upper_failure,minimum_upper_mm=minimum,hold_shift_mm=shift,hold_drift_deg=drift,final_top=top,
           worst=g.worst,terminal_guard=g.last,prefix_sha256=sha(prefix),model_provenance=provenance,candidate_sha256=sha(out/'candidate.npz'),
           model_contact_feedback_and_ik_copies_used=True,friction_is_unmeasured_hypothesis=True,native_guards_and_limits_unchanged=True,
           contact_feedback_used=a.contact_feedback,state_resets_after_initialization=int(bool(a.from_receiving)),cached_receiving_state=a.from_receiving,
           full_continuous_replay_required=bool(a.from_receiving),new_SAC_training_steps=0,devices_opened=0,physical_success=False,hardware_execution_allowed=False,next_rotation_included=False)
    save_json(out/'review.json',r);print(json.dumps({k:v for k,v in r.items() if k not in ('stages','worst','terminal_guard','model_provenance')}),flush=True)

if __name__=='__main__':main()
