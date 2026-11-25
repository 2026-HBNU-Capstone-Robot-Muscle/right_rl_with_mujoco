"""Source-bound hand/reentry and free-cube checks for one grasp observation."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,json,time
from pathlib import Path
import numpy as np,mujoco
from mujoco_playground import registry
from new_pair_grasp_plan_20261027 import NewPairGraspPlan,ENTRY,ROOTS,JOIN,OPEN,SETTLE,GRASP,HOLD,REOPEN,REMOVE,SEED,PROOF,hashes,loaded_digest
from fingertip_cad_model_20261025 import build
from loaded_quarter_model_policy_20261025 import correct_model
from loaded_quarter_reentry_20261025 import limits
from observed_book_model_policy import SOFT_JOINT_RESIDUAL_RAD
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS,TICK
from book_empty_preparation_plan import offsets,FRONTS
from book_support_contact_scope import derive_scope,allowed_initial_pairs
from screen_loaded_repeat_20261026 import cached_contacts
from sac_cube_model import set_cube_physics
def main():
    parser=argparse.ArgumentParser();parser.add_argument('request');parser.add_argument('output');args=parser.parse_args();out=Path(args.output)
    if out.exists():raise ValueError('Proof output must be new')
    began=time.perf_counter();plan=NewPairGraspPlan(args.request);raw=plan.source_position();rows=plan.plan(raw);before=hashes();seed=json.loads(SEED.read_bytes());up=np.array(seed['up_world']);cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);static=[];dynamic=[];loaded=[];hwlo,hwhi=limits();simids=np.array(SIM_TO_MOTOR_IDS)-1;native_lo=(hwlo[simids]-2048)*TICK;native_hi=(hwhi[simids]-2048)*TICK
    cached=None
    if out.resolve()!=PROOF.resolve() and PROOF.is_file():
        from new_pair_grasp_gate_20261027 import reusable_loaded_review
        cached=reusable_loaded_review(plan)
    for front in FRONTS:
        m,_=build(env,out.with_name(f'cycle_scene_{front:g}.xml'),book_projection_mm=front);range_policy=correct_model(m,raw);m.opt.timestep=.005;cube=m.geom('cube').id;cubegeoms=m.geom_bodyid==m.body('cube').id;original_ct=m.geom_contype[cubegeoms].copy();original_ca=m.geom_conaffinity[cubegeoms].copy();m.geom_contype[cubegeoms]=0;m.geom_conaffinity[cubegeoms]=0;hand={m.body('palm').id}
        for b in range(m.nbody):
            if int(m.body_parentid[b]) in hand:hand.add(b)
        support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};inspect=cached_contacts(m,hand,support);allowed=allowed_initial_pairs(derive_scope(m));d=mujoco.MjData(m);envelope={}
        def put(d,q):d.qpos[:16]=ticks_to_sim(q);d.qvel[:]=0;d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d)
        for off in offsets():
            a=raw+off;queue=plan.plan(a);put(d,a);initial,sp=inspect(d);previous=initial;bad=inc=selfmax=after=0.;clear=None;rootcount=0;clear_goal=None
            for k,v in initial.items():envelope[k]=max(envelope.get(k,0.),v)
            for phase,q in queue:
                put(d,q);su,sp=inspect(d);selfmax=max(selfmax,max(sp.values(),default=0.))
                if phase in (ENTRY,ROOTS):
                    bad=max(bad,max((v-initial.get(k,0.) for k,v in su.items()),default=0.));inc=max(inc,max((v-previous.get(k,0.) for k,v in su.items()),default=0.));previous=su
                    if phase==ROOTS:
                        rootcount+=1
                        if clear is None and max(su.values(),default=0.)<=.15:clear=rootcount;clear_goal=q[[7,11,15]].copy()
                else:after=max(after,max(su.values(),default=0.))
            passed=bool(set(initial)<=allowed and max(initial.values(),default=0.)<=5 and selfmax<=1e-6 and bad<=1e-6 and inc<=1e-6 and after<=.15 and clear is not None and clear<=70)
            static.append(dict(book_front_mm=front,offset_ticks=off.tolist(),passed=passed,initial_support_mm=initial,max_self_mm=selfmax,max_support_after_release_mm=after,initial_support_increase_mm=max(0,bad),step_support_increase_mm=max(0,inc),root_clear_sample=clear,root_clear_goal_ticks=clear_goal.tolist() if clear_goal is not None else None))
        print('STATIC',front,sum(r['passed'] for r in static if r['book_front_mm']==front),'/67',flush=True)
        gains=m.actuator_gainprm.copy();bias=m.actuator_biasprm.copy()
        for scale in (.5,1.,1.5):
            m.actuator_gainprm[:]=gains;m.actuator_biasprm[:]=bias;m.actuator_gainprm[:,0]*=scale;m.actuator_biasprm[:,1]*=scale;d=mujoco.MjData(m);put(d,raw);maxself=maxsup=maxtrack=aftertrack=maxrange=initialbad=0.;clear=None;root_time=0.;steps=0;scheduled=0.;finite=True
            for phase,q,duration in [('initial_hold',raw,.5)]+[(p,q,plan.SAMPLE_PERIOD) for p,q in rows]+[('final_hold',rows[-1][1],1.5)]:
                goal=ticks_to_sim(q);d.ctrl[:]=goal;scheduled+=duration;target=round(scheduled/.005);n=target-steps;steps=target
                for _ in range(n):
                    mujoco.mj_step(m,d);su,sp=inspect(d);maxself=max(maxself,max(sp.values(),default=0.));err=float(max(abs(d.qpos[:16]-goal)));maxtrack=max(maxtrack,err);ex=np.maximum(native_lo-d.qpos[:16],d.qpos[:16]-native_hi)
                    if phase in ('initial_hold',ENTRY):
                        j=int(np.flatnonzero(simids==3)[0]);extended=(range_policy['extended_lower_ticks']-2048)*TICK;ex[j]=max(extended-d.qpos[j]-SOFT_JOINT_RESIDUAL_RAD,d.qpos[j]-native_hi[j]);initialbad=max(initialbad,max((v-max(envelope.get(k,0.),.15 if k in allowed else 0.) for k,v in su.items()),default=0.))
                    elif phase==ROOTS:
                        root_time+=.005;initialbad=max(initialbad,max((v-max(envelope.get(k,0.),.15 if k in allowed else 0.) for k,v in su.items()),default=0.))
                        if clear is None and max(su.values(),default=0.)<=.15:clear=root_time
                    else:maxsup=max(maxsup,max(su.values(),default=0.));aftertrack=max(aftertrack,err)
                    maxrange=max(maxrange,float(max(ex)));finite=finite and bool(np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all())
            passed=bool(finite and maxself<=1e-6 and maxsup<=.15 and maxtrack<=.16 and aftertrack<=.12 and maxrange<=1e-6 and initialbad<=1e-6 and clear is not None and clear<=3.575);dynamic.append(dict(book_front_mm=front,gain_scale=scale,passed=passed,finite=finite,max_self_mm=maxself,max_support_after_release_mm=maxsup,max_tracking_rad=maxtrack,tracking_after_release_rad=aftertrack,unapproved_range_excess_rad=max(0,maxrange),initial_support_increase_mm=max(0,initialbad),root_clear_seconds=clear));print('HAND RESPONSE',front,scale,passed,flush=True)
        if cached is not None:
            loaded.extend(r.copy() for r in cached if r['book_front_mm']==front)
            print('LOADED REVIEW REUSED',front,'9 fixed-open cases; fresh recovery was rerun',flush=True)
            continue
        m.geom_contype[cubegeoms]=original_ct;m.geom_conaffinity[cubegeoms]=original_ca;set_cube_physics(m,.055,.1);m.opt.timestep=.002;m.opt.gravity[:]=-9.81*up;positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004);tips={f:m.geom(f+'_tip').id for f in ('th','mf')};opened=next(q for p,q in rows if p==SETTLE);loaded_rows=[(p,q) for p,q in rows if p in (GRASP,HOLD,REOPEN,REMOVE)]
        def observe(d):
            su,sp=inspect(d);cp=0.;external=False;contacts=set();hand_contact=False
            for c in d.contact:
                a,b=int(c.geom1),int(c.geom2)
                if cube in (a,b):
                    other=b if a==cube else a
                    if int(m.geom_bodyid[other]) in hand:
                        cp=max(cp,max(0.,-float(c.dist)*1000))
                        if c.dist<.00015:contacts.add(other);hand_contact=True
                    elif other in support and c.dist<.0004:external=True
            return max(sp.values(),default=0.),max(su.values(),default=0.),cp,external,{f:g in contacts for f,g in tips.items()},hand_contact
        for scale in (.5,1.,1.5):
            for friction in (.2,.3,.5):
                m.actuator_gainprm[:]=gains;m.actuator_biasprm[:]=bias;m.actuator_gainprm[:,0]*=scale;m.actuator_biasprm[:,1]*=scale;m.geom_friction[cube,0]=friction;d=mujoco.MjData(m);put(d,opened)
                for _ in range(500):mujoco.mj_step(m,d)
                d.qpos[-7:]=seed['settled_cube_qpos'];d.qvel[-6:]=0;raise_mm=0
                for raise_mm in range(31):
                    d.qpos[-7:-4]=np.array(seed['settled_cube_qpos'][:3])+up*raise_mm*.001;mujoco.mj_forward(m,d)
                    if observe(d)[2]<=1e-6:break
                d.qpos[-7:-4]+=up*.001;mujoco.mj_forward(m,d);maxself=maxsup=maxcp=maxtrack=maxrange=0.;external=False;finite=True;hold_grasp=0.;stable=0.;sample=0
                def track(goal):
                    nonlocal maxself,maxsup,maxcp,maxtrack,maxrange,external,finite
                    sp,su,cp,ext,tip,contact=observe(d);maxself=max(maxself,sp);maxsup=max(maxsup,su);maxcp=max(maxcp,cp);maxtrack=max(maxtrack,float(max(abs(d.qpos[:16]-goal))));maxrange=max(maxrange,float(max(np.maximum(native_lo-d.qpos[:16],d.qpos[:16]-native_hi))));external=external or ext;finite=finite and bool(np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all());return tip,contact
                for _ in range(1000):mujoco.mj_step(m,d);track(ticks_to_sim(opened))
                settled=d.qpos[-7:].copy();steps=0
                for sample,(phase,q) in enumerate(loaded_rows,1):
                    goal=ticks_to_sim(q);d.ctrl[:]=goal;target=round(sample*.075/.002);n=target-steps;steps=target
                    for _ in range(n):mujoco.mj_step(m,d);tip,contact=track(goal)
                    stable=stable+.075 if all(tip.values()) else 0.
                    if phase==HOLD:hold_grasp=max(hold_grasp,stable)
                sp,su,cp,ext,tip,contact=observe(d);passed=bool(finite and maxself<=1e-6 and maxsup<=.15 and maxcp<=1 and maxtrack<=.18 and maxrange<=1e-6 and not external and contact);loaded.append(dict(book_front_mm=front,gain_scale=scale,friction=friction,passed=passed,finite=finite,max_self_mm=maxself,max_support_mm=maxsup,max_cube_mm=maxcp,max_tracking_rad=maxtrack,unapproved_range_excess_rad=max(0,maxrange),external_cube_contact=external,final_cube_supported_by_hand=contact,two_tip_hold_seconds=hold_grasp,starting_height_raise_mm=raise_mm+1,settled_cube_qpos=settled.tolist(),final_cube_qpos=d.qpos[-7:].tolist()));print('LOADED',front,scale,friction,passed,'hold',round(hold_grasp,2),flush=True)
    after=hashes();nominal=[r for r in loaded if r['book_front_mm']==-10 and r['gain_scale']==1 and r['friction']==.3];accepted=bool(before==after and len(static)==201 and len(dynamic)==9 and len(loaded)==27 and all(r['passed'] for r in static+dynamic+loaded) and nominal[0]['two_tip_hold_seconds']>=3)
    result=dict(status='new_pair_grasp_model_review',accepted_for_operator_grasp_trial=accepted,plan_version=plan.PLAN_VERSION,source_sha256=plan.SOURCE_SHA,plan_sha256=plan.plan_digest(raw),loaded_cycle_sha256=loaded_digest(plan),loaded_review_reused=cached is not None,loaded_review_source=str(PROOF) if cached is not None else None,loaded_initial_state='fixed_open_pose_settled_1s',start_ticks=raw.tolist(),last_goal_ticks=rows[-1][1].tolist(),mapping=list(SIM_TO_MOTOR_IDS),code_hashes=after,code_unchanged=before==after,static_screens=static,dynamic_sensitivity=dynamic,loaded_physics=loaded,goal_step_ticks=2,sample_period_s=.075,initial_support_bound_mm=5,root_escape_goals=70,root_escape_seconds=3.5,support_after_release_mm=.15,self_contact_mm=1e-6,hardware_tracking_stop_rad=.18,passive_id4_reentry_only=True,physical_grasp_verified=False,physical_rotation_verified=False,cube_pose_and_friction_hypotheses=True,finite_stiffness_contact_forces_preserved=True,hardware_commands_sent=0,elapsed_seconds=time.perf_counter()-began);out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print('ACCEPTED',accepted,flush=True)
if __name__=='__main__':main()
