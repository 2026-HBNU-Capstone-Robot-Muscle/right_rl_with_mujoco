"""Offline recovery screen for fresh OFF reads; never authorizes hardware.

Uses the already established passive ID4 admission and monotonic reentry.
Does not widen active goals, contact criteria, or any hardware health stop.
"""
import json
import hashlib
from pathlib import Path
import numpy as np
from new_pair_grasp_plan_20261027 import NewPairGraspPlan,ENTRY,ROOTS,OPEN,SETTLE
from observed_book_model_policy import admission,healthy_off
from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK,ticks_to_sim
from horizontal_empty_recovery_20261027 import RING_CLEAR,needs_ring_clearance
from rewired_initial_pair_scope_20261027 import InitialPairScope,PAIR
from horizontal_thumb_book_numeric_20261027 import PAIR as THUMB_BOOK,PHASES as THUMB_PHASES,allowance as thumb_allowance

from calibrated_empty_ring_margin_20261028 import PHASE as RING_MARGIN,rows as margin_rows
ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'repeat_off_readonly_20261027_133354_321429/result.json'
OUT=ROOT/'right_hand_rotation_20261027/current_recovery_133354'


class OfflineRecovery(NewPairGraspPlan):
    PLAN_VERSION='yaw_current_off_recovery_diagnostic_20261027'
    validate_admission=staticmethod(admission)
    validate_off_state=staticmethod(healthy_off)
    def plan(self,raw):
        return margin_rows(self,raw)


def prepare():
    report=json.loads(SOURCE.read_bytes())
    assert report['status']=='off_snapshots_saved' and report['servo_writes']==0 and report['torque_changes']==0 and len(report['snapshots'])==3
    assert report['maximum_drift_ticks']<=2
    for state in report['snapshots']:healthy_off(state)
    OUT.mkdir(exist_ok=True);state=report['snapshots'][-1]
    request=dict(plan_version=OfflineRecovery.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=state,source=SOURCE.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),hardware_execution_allowed=False)
    (OUT/'request.json').write_text(json.dumps(request,indent=2)+'\n',encoding='utf-8')
    return OfflineRecovery(OUT/'request.json')


def main():
    global OUT
    import os,time,argparse
    parser=argparse.ArgumentParser();parser.add_argument('--request');parser.add_argument('--output-folder');args=parser.parse_args()
    original_request=None
    if args.output_folder:
        OUT=(ROOT/args.output_folder).resolve();OUT.relative_to(ROOT);OUT.mkdir(parents=True,exist_ok=True)
    os.environ.setdefault('MUJOCO_GL','egl')
    import mujoco
    from mujoco_playground import registry
    from fingertip_cad_model_20261025 import build
    from loaded_quarter_model_policy_20261025 import correct_model
    from loaded_quarter_reentry_20261025 import limits
    from book_empty_preparation_plan import FRONTS,offsets
    from book_support_contact_scope import derive_scope,allowed_initial_pairs
    from screen_loaded_repeat_20261026 import cached_contacts
    began=time.perf_counter()
    if args.request:
        original_request=Path(args.request).resolve();original_request.relative_to(ROOT);request=json.loads(original_request.read_bytes());request['plan_version']=OfflineRecovery.PLAN_VERSION;normalized=OUT/'request.json';normalized.write_text(json.dumps(request,indent=2)+'\n',encoding='utf-8');plan=OfflineRecovery(normalized)
    else:plan=prepare()
    raw=plan.source_position();rows=[(p,q) for p,q in plan.plan(raw) if p in (ENTRY,RING_CLEAR,RING_MARGIN,ROOTS,OPEN,SETTLE)];self_scope_enabled=needs_ring_clearance(raw)
    np.savez_compressed(OUT/'prefix_path.npz',start=raw,goals=np.asarray([q for _,q in rows]),phases=np.asarray([p for p,_ in rows]))
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);statics=[];dynamics=[]
    low,high=limits();simids=np.array(SIM_TO_MOTOR_IDS)-1;native_low=(low[simids]-2048)*TICK;native_high=(high[simids]-2048)*TICK
    for front in FRONTS:
        m,_=build(env,OUT/f'scene_{front:g}.xml',book_projection_mm=front);policy=correct_model(m,raw);m.opt.timestep=.005
        cubemask=m.geom_bodyid==m.body('cube').id;m.geom_contype[cubemask]=0;m.geom_conaffinity[cubemask]=0
        hand={m.body('palm').id}
        for b in range(m.nbody):
            if int(m.body_parentid[b]) in hand:hand.add(b)
        support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
        inspect=cached_contacts(m,hand,support);allowed=allowed_initial_pairs(derive_scope(m));envelope={}
        def put(d,q):
            d.qpos[:16]=ticks_to_sim(q);d.ctrl[:]=d.qpos[:16];d.qvel[:]=0;d.qpos[-7:]=[0,0,2,1,0,0,0];mujoco.mj_forward(m,d)
        for offset in offsets():
            a=raw+offset;queue=[(p,q) for p,q in plan.plan(a) if p in (ENTRY,RING_CLEAR,RING_MARGIN,ROOTS,OPEN,SETTLE)];d=mujoco.MjData(m);put(d,a);initial,sp=inspect(d);selfscope=InitialPairScope(sp,self_scope_enabled)
            for k,v in initial.items():envelope[k]=max(envelope.get(k,0.),v)
            previous=initial;maxself=max(selfscope.filtered('initial_hold',sp).values(),default=0.);maxafter=maxincrease=maxstep=0.;clear=None;rootn=0;worst={};ringn=0;ringclear=None;ringgoal=None;last_ring_depth=0.;pair_previous=selfscope.initial_depth;pair_step_increase=0.;thumb_commanded=initial.get(THUMB_BOOK,0.)
            for phase,q in queue:
                put(d,q);su,sp=inspect(d);maxself=max(maxself,max(selfscope.filtered(phase,sp).values(),default=0.));thumb_commanded=max(thumb_commanded,su.get(THUMB_BOOK,0.))
                if phase in (ENTRY,RING_CLEAR):
                    depth=sp.get(PAIR,0.);pair_step_increase=max(pair_step_increase,depth-pair_previous);pair_previous=depth
                    if phase==RING_CLEAR:
                        ringn+=1;last_ring_depth=depth
                        if ringclear is None and depth<=1e-6:ringclear=ringn;ringgoal=int(q[13])
                for k,v in sp.items():worst[k]=max(worst.get(k,0.),v)
                if phase in (ENTRY,RING_MARGIN,ROOTS):
                    maxincrease=max(maxincrease,max((v-initial.get(k,0.) for k,v in su.items()),default=0.));maxstep=max(maxstep,max((v-previous.get(k,0.) for k,v in su.items()),default=0.));previous=su
                    if phase==ROOTS:
                        rootn+=1
                        if clear is None and max(su.values(),default=0.)<=.15:clear=rootn
                else:maxafter=max(maxafter,max(su.values(),default=0.))
            passed=bool(thumb_commanded<=1e-6 and selfscope.valid and (not self_scope_enabled or ringclear is not None and ringclear<=48 and last_ring_depth<=1e-6 and pair_step_increase<=1e-6) and set(initial)<=allowed and max(initial.values(),default=0.)<=5 and maxself<=1e-6 and maxincrease<=1e-6 and maxstep<=1e-6 and maxafter<=.15 and clear is not None and clear<=70)
            root_rows=[q for p,q in queue if p==ROOTS];clear_goal=root_rows[clear-1][[7,11,15]].tolist() if clear is not None else None
            statics.append(dict(book_front_mm=front,offset_ticks=offset.tolist(),passed=passed,max_self_mm=maxself,worst_self_pairs_mm=worst,max_support_after_release_mm=maxafter,initial_support_increase_mm=max(0,maxincrease),step_support_increase_mm=max(0,maxstep),root_clear_sample=clear,root_clear_goal_ticks=clear_goal,initial_pair_scope_enabled=self_scope_enabled,initial_pair_scope_valid=selfscope.valid,initial_pair_mm=selfscope.initial_depth,maximum_raw_self_mm=selfscope.maximum_raw,initial_pair_increase_mm=max(0,selfscope.maximum_increase),initial_pair_step_increase_mm=max(0,pair_step_increase),ring_clear_sample=ringclear,ring_clear_goal_ticks=ringgoal,last_ring_pair_mm=last_ring_depth,commanded_thumb_book_mm=thumb_commanded))
        print('STATIC',front,sum(v['passed'] for v in statics if v['book_front_mm']==front),'/67',flush=True)
        gains=m.actuator_gainprm.copy();bias=m.actuator_biasprm.copy()
        for scale in (.5,1.,1.5):
            m.actuator_gainprm[:]=gains;m.actuator_biasprm[:]=bias;m.actuator_gainprm[:,0]*=scale;m.actuator_biasprm[:,1]*=scale;d=mujoco.MjData(m);put(d,raw);_,initial_self=inspect(d);selfscope=InitialPairScope(initial_self,self_scope_enabled);selfmax=supmax=trackmax=posttrack=range_excess=inc=0.;root_time=0.;clear=None;scheduled=0.;steps=0;last_ring_depth=0.;ring_actual=None;raw_thumb_held=0.
            for phase,q,duration in [('initial_hold',raw,.5)]+[(p,q,.075) for p,q in rows]+[('final_hold',rows[-1][1],1.5)]:
                d.ctrl[:]=ticks_to_sim(q);scheduled+=duration;target=round(scheduled/.005)
                for _ in range(target-steps):
                    mujoco.mj_step(m,d);su,sp=inspect(d);selfmax=max(selfmax,max(selfscope.filtered(phase,sp).values(),default=0.));error=float(max(abs(d.qpos[:16]-d.ctrl)));trackmax=max(trackmax,error);ex=np.maximum(native_low-d.qpos[:16],d.qpos[:16]-native_high)
                    if phase==RING_CLEAR:last_ring_depth=sp.get(PAIR,0.);ring_actual=float(d.qpos[int(np.flatnonzero(simids==13)[0])]/TICK+2048)
                    if phase in ('initial_hold',ENTRY):
                        j=int(np.flatnonzero(simids==3)[0]);ex[j]=max((policy['extended_lower_ticks']-2048)*TICK-d.qpos[j]-TICK,d.qpos[j]-native_high[j])
                    if phase in ('initial_hold',ENTRY,RING_MARGIN,ROOTS):
                        if phase in THUMB_PHASES:raw_thumb_held=max(raw_thumb_held,su.get(THUMB_BOOK,0.))
                        inc=max(inc,max((v-max(envelope.get(k,0.),.15 if k in allowed else thumb_allowance(raw,phase,k)) for k,v in su.items()),default=0.))
                        if phase==ROOTS:
                            root_time+=.005
                            if clear is None and max(su.values(),default=0.)<=.15:clear=root_time
                    else:supmax=max(supmax,max(su.values(),default=0.));posttrack=max(posttrack,error)
                    range_excess=max(range_excess,float(max(ex)))
                steps=target
            passed=bool(selfscope.valid and (not self_scope_enabled or last_ring_depth<=1e-6 and selfscope.maximum_increase<=1e-6) and selfmax<=1e-6 and supmax<=.15 and trackmax<=.16 and posttrack<=.12 and range_excess<=1e-6 and inc<=1e-6 and clear is not None and clear<=3.575)
            row=dict(book_front_mm=front,gain_scale=scale,passed=passed,max_self_mm=selfmax,max_support_mm=supmax,max_tracking_rad=trackmax,post_release_tracking_rad=posttrack,unapproved_range_excess_rad=max(0,range_excess),initial_support_increase_mm=max(0,inc),root_clear_seconds=clear,initial_pair_scope_enabled=self_scope_enabled,initial_pair_scope_valid=selfscope.valid,initial_pair_mm=selfscope.initial_depth,maximum_raw_self_mm=selfscope.maximum_raw,initial_pair_increase_mm=max(0,selfscope.maximum_increase),last_ring_pair_mm=last_ring_depth,ring_rotation_after_clearance_ticks=ring_actual,raw_held_thumb_book_mm=raw_thumb_held);dynamics.append(row);print('DYNAMIC',json.dumps(row),flush=True)
    prefix_digest=hashlib.sha256(json.dumps([(p,q.tolist()) for p,q in rows],separators=(',',':')).encode()).hexdigest()
    result=dict(status='current_off_empty_recovery_diagnostic',source=(original_request if original_request else SOURCE).relative_to(ROOT).as_posix(),original_request=original_request.relative_to(ROOT).as_posix() if original_request else None,original_request_sha256=hashlib.sha256(original_request.read_bytes()).hexdigest() if original_request else None,start_ticks=raw.tolist(),goals=len(rows),prefix_sha256=prefix_digest,static_cases=statics,dynamic_cases=dynamics,all_passed=all(r['passed'] for r in statics+dynamics),hardware_execution_allowed=False,devices_opened=False,elapsed_seconds=time.perf_counter()-began)
    (OUT/'review.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print('COMPLETE',result['all_passed'],flush=True)
if __name__=='__main__':main()
