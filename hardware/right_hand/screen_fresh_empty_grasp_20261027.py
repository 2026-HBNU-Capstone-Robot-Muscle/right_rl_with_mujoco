"""Same support/health bounds, separate new EMPTY route geometry/dynamics proof."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,json,time
from pathlib import Path
import numpy as np,mujoco
from mujoco_playground import registry
from fresh_empty_grasp_plan_20261027 import FreshEmptyGraspPlan,RING_PHASE,ROOT_PHASE,GRASP,ROOT,hashes
from fingertip_cad_model_20261025 import build
from loaded_quarter_model_policy_20261025 import correct_model
from book_empty_preparation_plan import offsets,FRONTS
from book_support_contact_scope import derive_scope,allowed_initial_pairs
from book_passive_support_reentry_policy import INITIAL_SUPPORT_DEPTH_MM,ROOT_ESCAPE_GOALS,ROOT_ESCAPE_SECONDS,INITIAL_MODEL_TRACKING_RAD
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS
from screen_loaded_repeat_20261026 import cached_contacts
from book_grasp_trial_plan import RING_SELF_PAIRS,MAX_INITIAL_THUMB_SELF_MM

def main():
    parser=argparse.ArgumentParser();parser.add_argument('request');parser.add_argument('output');parser.add_argument('--nominal-only',action='store_true');args=parser.parse_args();out=Path(args.output)
    if out.exists():raise ValueError('Use a new proof output')
    began=time.perf_counter();plan=FreshEmptyGraspPlan(args.request);raw=plan.source_position();before=hashes();cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);statics=[];dynamics=[];camera_cases=[];manifests=[]
    for front in ((-10.,) if args.nominal_only else FRONTS):
        m,manifest=build(env,out.with_name(f'scene_front_{front:g}.xml'),book_projection_mm=front);correct_model(m,raw);m.opt.timestep=.005;cube_geoms=m.geom_bodyid==m.body('cube').id;m.geom_contype[cube_geoms]=0;m.geom_conaffinity[cube_geoms]=0;hand={m.body('palm').id}
        for b in range(m.nbody):
            if int(m.body_parentid[b]) in hand:hand.add(b)
        support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};inspect=cached_contacts(m,hand,support);allowed=allowed_initial_pairs(derive_scope(m));manifests.append(manifest);envelope={};self_envelope={};base_rows=plan.plan(raw)
        def put(d,q):d.qpos[:16]=ticks_to_sim(q);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];d.qvel[:]=0;mujoco.mj_forward(m,d)
        # Reuse the existing INITIAL ring/palm escape scope. Its OFF +/-4
        # envelope also covers the established short passive enable hold.
        # This does not change any geometry, force or post-release tolerance.
        for off in offsets():
            probe=mujoco.MjData(m);put(probe,raw+off);su,sp=inspect(probe)
            if not set(sp)<=set(RING_SELF_PAIRS) or max(sp.values(),default=0.)>MAX_INITIAL_THUMB_SELF_MM:raise ValueError('OFF self contact outside existing ring escape scope')
            for key,value in sp.items():self_envelope[key]=max(self_envelope.get(key,0.),value)
        for off in ([np.zeros(16,int)] if args.nominal_only else offsets()):
            start=raw+off;rows=plan.plan(start);d=mujoco.MjData(m);put(d,start);initial,self_initial=inspect(d);bound=dict(initial);previous=dict(initial);self_bound=dict(self_initial);self_previous=dict(self_initial);spmax=after=unapproved=increase=self_after=self_bad=self_increase=0.;self_clear=0 if not self_initial else None;last_ring_contact=0 if self_initial else -1;clear=None;root_count=sum(p==ROOT_PHASE for p,_ in rows)
            for key,value in initial.items():envelope[key]=max(envelope.get(key,0.),value)
            root_step=0
            for i,(phase,q) in enumerate(rows):
                put(d,q);su,sp=inspect(d);spmax=max(spmax,max(sp.values(),default=0.))
                if phase==RING_PHASE:
                    self_bad=max(self_bad,max((v-self_bound.get(k,0.) for k,v in sp.items()),default=0.));self_increase=max(self_increase,max((v-self_previous.get(k,0.) for k,v in sp.items()),default=0.));self_previous=sp
                    if self_clear is None and max(sp.values(),default=0.)<=1e-6:self_clear=i+1
                    if max(sp.values(),default=0.)>1e-6:last_ring_contact=i+1
                else:self_after=max(self_after,max(sp.values(),default=0.))
                if phase in (RING_PHASE,ROOT_PHASE):
                    unapproved=max(unapproved,max((v-bound.get(k,0.) for k,v in su.items()),default=0.));increase=max(increase,max((v-previous.get(k,0.) for k,v in su.items()),default=0.));previous=su
                    if phase==ROOT_PHASE:
                        root_step+=1
                        if clear is None and max(su.values(),default=0.)<=.15:clear=root_step
                else:after=max(after,max(su.values(),default=0.))
            passed=bool(set(self_initial)<=set(RING_SELF_PAIRS) and max(self_initial.values(),default=0.)<=MAX_INITIAL_THUMB_SELF_MM and self_after<=1e-6 and self_bad<=1e-6 and self_increase<=1e-6 and self_clear is not None and self_clear<=40 and set(initial)<=allowed and max(initial.values(),default=0.)<=INITIAL_SUPPORT_DEPTH_MM and after<=.15 and unapproved<=1e-6 and increase<=1e-6 and clear is not None and clear<=ROOT_ESCAPE_GOALS)
            ring_goals=[q for phase,q in rows if phase==RING_PHASE];settled_clear=max(0,last_ring_contact+1) if last_ring_contact<len(ring_goals) else None;clear_goal=start if settled_clear==0 else ring_goals[settled_clear-1] if settled_clear is not None else None;passed=passed and settled_clear is not None
            statics.append(dict(book_front_mm=front,offset_ticks=off.tolist(),passed=passed,initial_support_mm=initial,initial_ring_self_mm=self_initial,max_self_mm=spmax,max_self_after_ring_mm=self_after,initial_ring_excess_mm=max(0.,self_bad),initial_ring_step_increase_mm=max(0.,self_increase),ring_clear_sample=settled_clear,ring_clear_goal_ticks=int(clear_goal[14]) if clear_goal is not None else None,ring_clear_dip_goal_ticks=int(clear_goal[12]) if clear_goal is not None else None,max_support_after_release_mm=after,unapproved_support_increase_mm=max(0.,unapproved),maximum_step_increase_mm=max(0.,increase),support_clear_sample=clear,root_samples=root_count))
        print('STATIC',front,sum(v['passed'] for v in statics if v['book_front_mm']==front),'/',sum(v['book_front_mm']==front for v in statics),flush=True)
        gains=m.actuator_gainprm.copy();bias=m.actuator_biasprm.copy();joints=m.actuator_trnid[:,0];lo=np.maximum(m.jnt_range[joints,0],m.actuator_ctrlrange[:,0]);hi=np.minimum(m.jnt_range[joints,1],m.actuator_ctrlrange[:,1])
        for scale in ((1.,) if args.nominal_only else (.5,1.,1.5)):
            for with_gap in (False,True):
                m.actuator_gainprm[:]=gains;m.actuator_biasprm[:]=bias;m.actuator_gainprm[:,0]*=scale;m.actuator_biasprm[:,1]*=scale
                d=mujoco.MjData(m);put(d,raw);d.ctrl[:]=ticks_to_sim(raw);root_end=next(i for i,(p,_) in enumerate(base_rows) if p!=ROOT_PHASE);segments=[('initial_hold',raw,.5)]+[(phase,q,plan.SAMPLE_PERIOD) for phase,q in base_rows]+[('final_hold',base_rows[-1][1],20*plan.SAMPLE_PERIOD)]
                # A five-second pause while the initial support is separating
                # models the existing EMPTY image-gap hold. No goals advance.
                late_gap=1+next(i for i,(phase,_) in enumerate(base_rows) if phase==GRASP)+sum(phase==GRASP for phase,_ in base_rows)//2
                pause_points={10,late_gap} if with_gap else set();elapsed=0.;scheduled=0.;steps=0;maxself=maxafter=maxtrack=posttrack=maxexcess=bad=self_after=self_bad=0.;clear_t=None;finite=True;initial_bound=dict(envelope);pause_duration=0.;previous_goal=ticks_to_sim(raw);worst_self=None;ring_seconds=sum(p==RING_PHASE for p,_ in base_rows)*plan.SAMPLE_PERIOD
                for si,(phase,q,duration) in enumerate(segments):
                    if si in pause_points:
                        count=round(5./m.opt.timestep);pause_duration+=5.
                        for _ in range(count):
                            d.ctrl[:]=previous_goal;mujoco.mj_step(m,d);su,sp=inspect(d);maxself=max(maxself,max(sp.values(),default=0.));tracking=float(np.max(abs(d.qpos[:16]-previous_goal)));maxtrack=max(maxtrack,tracking);maxexcess=max(maxexcess,float(max(0.,np.max(lo-d.qpos[:16]),np.max(d.qpos[:16]-hi))));finite=finite and bool(np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all())
                            if phase in ('initial_hold',RING_PHASE):self_bad=max(self_bad,max((v-self_envelope.get(k,0.) for k,v in sp.items()),default=0.))
                            else:self_after=max(self_after,max(sp.values(),default=0.))
                            if phase in ('initial_hold',RING_PHASE,ROOT_PHASE):bad=max(bad,max((v-max(initial_bound.get(k,0.),.15 if k in allowed else 0.) for k,v in su.items()),default=0.))
                            else:maxafter=max(maxafter,max(su.values(),default=0.));posttrack=max(posttrack,tracking)
                        elapsed+=5.
                    goal=ticks_to_sim(q);d.ctrl[:]=goal;scheduled+=duration;target_steps=round(scheduled/m.opt.timestep);count=target_steps-steps;steps=target_steps
                    for _ in range(count):
                        mujoco.mj_step(m,d);elapsed+=m.opt.timestep;su,sp=inspect(d);tr=float(np.max(abs(d.qpos[:16]-goal)));maxtrack=max(maxtrack,tr)
                        if max(sp.values(),default=0.)>maxself:worst_self=dict(phase=phase,queue_sample=si,time_s=elapsed,pairs=sp,target_ticks=q.tolist(),actual_sim_angles=d.qpos[:16].tolist())
                        maxself=max(maxself,max(sp.values(),default=0.));maxexcess=max(maxexcess,float(max(0.,np.max(lo-d.qpos[:16]),np.max(d.qpos[:16]-hi))))
                        if phase in ('initial_hold',RING_PHASE):self_bad=max(self_bad,max((v-self_envelope.get(k,0.) for k,v in sp.items()),default=0.))
                        else:self_after=max(self_after,max(sp.values(),default=0.))
                        if phase in ('initial_hold',RING_PHASE,ROOT_PHASE):
                            bad=max(bad,max((v-max(initial_bound.get(k,0.),.15 if k in allowed else 0.) for k,v in su.items()),default=0.))
                            if phase==ROOT_PHASE and clear_t is None and max(su.values(),default=0.)<=.15:clear_t=elapsed-.5-pause_duration-ring_seconds
                        else:maxafter=max(maxafter,max(su.values(),default=0.));posttrack=max(posttrack,tr)
                        finite=finite and bool(np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all())
                    previous_goal=goal
                passed=bool(finite and self_after<=1e-6 and self_bad<=1e-6 and maxafter<=.15 and maxtrack<=INITIAL_MODEL_TRACKING_RAD and posttrack<=.12 and maxexcess<=1e-6 and bad<=1e-6 and clear_t is not None and clear_t<=ROOT_ESCAPE_SECONDS+plan.SAMPLE_PERIOD)
                row=dict(book_front_mm=front,gain_scale=scale,camera_hold_seconds=pause_duration,passed=passed,finite=finite,max_self_mm=maxself,max_self_after_ring_mm=self_after,initial_ring_excess_mm=max(0.,self_bad),initial_ring_envelope_mm=self_envelope,max_support_after_release_mm=maxafter,max_tracking_rad=maxtrack,tracking_after_root_release_rad=posttrack,native_range_excess_rad=maxexcess,initial_support_bound_excess_mm=max(0.,bad),support_clear_time_excluding_image_pause_s=clear_t,worst_self=worst_self)
                (camera_cases if with_gap else dynamics).append(row);print('DYNAMIC',front,scale,pause_duration,passed,flush=True)
    after=hashes();accepted=not args.nominal_only and before==after and len(statics)==201 and len(dynamics)==9 and len(camera_cases)==9 and all(r['passed'] for r in statics+dynamics+camera_cases)
    result=dict(status='fresh_empty_current_pose_screen',accepted_for_empty_only=accepted,diagnostic_nominal_only=args.nominal_only,plan_version=plan.PLAN_VERSION,source_sha256=plan.SOURCE_SHA,plan_sha256=plan.plan_digest(raw),start_ticks=raw.tolist(),last_goal_ticks=base_rows[-1][1].tolist(),mapping=list(SIM_TO_MOTOR_IDS),code_hashes=after,code_unchanged=before==after,static_screens=statics,dynamic_sensitivity=dynamics,camera_gap_hold_sensitivity=camera_cases,model_manifests=manifests,goal_step_ticks=2,root_initial_average_step_ticks=plan.ROOT_INITIAL_AVERAGE_STEP,root_initial_travel_ticks=plan.ROOT_INITIAL_TRAVEL,root_tail_goal_step_ticks=plan.ROOT_TAIL_GOAL_STEP,sample_period_s=plan.SAMPLE_PERIOD,initial_support_bound_mm=INITIAL_SUPPORT_DEPTH_MM,initial_ring_self_bound_mm=MAX_INITIAL_THUMB_SELF_MM,initial_ring_pairs=list(RING_SELF_PAIRS),root_escape_goals=ROOT_ESCAPE_GOALS,root_escape_seconds=ROOT_ESCAPE_SECONDS,self_contact_tolerance_mm=1e-6,support_after_release_tolerance_mm=.15,actual_tracking_stop_rad=.18,cube_disabled_entire_screen=True,hardware_commands_sent=0,physical_calibration_verified=False,loaded_motion_allowed=False,elapsed_seconds=time.perf_counter()-began);out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print('ACCEPTED EMPTY',accepted,flush=True)
if __name__=='__main__':main()
