"""Device-free full-route replay for preparation, placement hold and grasp.

The cube is disabled in this hand clearance screen. It does not certify cube
contact, placement, grip force or rotation. Targets after PLACE follow the complete proposed reference. The Cube is not present in this hand-route screen.
Initial support bounds distinguish the +/-4 tick start and fixed enable hold;
contact forces and hardware current/voltage/tracking stops are unchanged.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse
import json
import time
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from book_support_model_20261024 import build_book
from book_empty_preparation_plan import (FRONTS,LIFT,
    SUPPORT_ESCAPE_STATIC_GOALS,SUPPORT_ESCAPE_DYNAMIC_SECONDS,ALLOWED_SUPPORT_PAIRS,
    offsets,code_hashes)
from book_grasp_trial_plan import (BookGraspPlan,extra_hashes,LOADED_PHASE,THUMB_RELEASE,RING_RELEASE,RING_SELF_PAIRS,
    SELF_PAIRS,MAX_INITIAL_THUMB_SELF_MM,THUMB_ESCAPE_GOALS,SUPPORT_ROOT_ESCAPE_GOALS,
    INITIAL_CONTACT_TRACKING_RAD,MAX_INITIAL_SUPPORT_MM,ROOT_ESCAPE_SECONDS)
from book_passive_support_reentry_policy import policy as support_reentry_policy
from screen_book_empty_preparation import escape_fields
from book_support_contact_scope import derive_scope,load_scope
from initial_contact_escape import ContactEscape
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS
from observed_book_model_policy import (correct_model,classify_self_contacts,policy,
    CONTACT_PAIR,SOFT_JOINT_RESIDUAL_RAD)


from fingertip_cad_model_20261025 import build as build_cad
from reverse_start_empty_plan_20261025 import ReverseStartPlan as BookGraspPlan,extra_hashes,END_HOLD
from reverse_place_capture_plan_20261025 import ReversePlacePlan as BookGraspPlan,extra_hashes,BASE_RELEASE,CASING_PAIR,CASING_INITIAL_LIMIT_MM,INITIAL_SUPPORT_SCOPE
THUMB_RELEASE=BASE_RELEASE
SELF_PAIRS=SELF_PAIRS+(CASING_PAIR,)
LOADED_PHASE=END_HOLD
from reverse_quarter_path_empty_plan_20261025 import ReverseQuarterEmptyPlan as BookGraspPlan,extra_hashes,START_HOLD,TURN_PHASE,motion_goals
from loaded_quarter_trial_plan_20261025 import LoadedQuarterTrialPlan as BookGraspPlan,extra_hashes,motion_goals,START_HOLD,TURN_PHASE,END_HOLD,RETURN_PHASE,EXTRA_RING_PAIR,EXTRA_RING_INITIAL_LIMIT_MM,INITIAL_SUPPORT_SCOPE,validate_experiment_basis,THUMB_NEIGHBOR_PAIR,THUMB_NEIGHBOR_LIMIT_MM,INITIAL_SELF_SCOPE
SELF_PAIRS=SELF_PAIRS+(THUMB_NEIGHBOR_PAIR,)
RING_SELF_PAIRS=RING_SELF_PAIRS+(EXTRA_RING_PAIR,)
SELF_PAIRS=SELF_PAIRS+(EXTRA_RING_PAIR,)
from loaded_quarter_reentry_20261025 import cmc_native_min
from loaded_quarter_model_policy_20261025 import policy,correct_model
from quarter_finish_empty_plan_20261025 import QuarterFinishEmptyPlan as BookGraspPlan,TURN_PHASE,END_HOLD,FINISH_PHASE,empty_basis,finish_goals
from support_photo_contact_resolution_20261025 import classify as classify_photo_initial,metadata as photo_resolution_metadata
def clear_goal(start,rows,at,j):
    if at is None:return None
    return int(start[j] if at==0 else rows[int(at)-1][1][j])
SCALES=(.5,1.,1.5)


def cached_contacts(m,hand,support):
    is_hand=np.isin(m.geom_bodyid,list(hand));is_support=np.zeros(m.ngeom,dtype=bool)
    is_support[list(support)]=True;names=[m.geom(g).name for g in range(m.ngeom)]
    def inspect(d):
        self_pairs={};support_pairs={}
        for index in np.flatnonzero(d.contact.dist<0):
            a,b=int(d.contact.geom1[index]),int(d.contact.geom2[index])
            depth=-float(d.contact.dist[index])*1000
            if is_hand[a] and is_hand[b]:dest=self_pairs
            elif (is_support[a] and is_hand[b]) or (is_support[b] and is_hand[a]):dest=support_pairs
            else:continue
            pair='|'.join(sorted((names[a],names[b])));dest[pair]=max(dest.get(pair,0.),depth)
        return support_pairs,self_pairs
    return inspect


def passed(row,dynamic=False):
    keys=('max_self_after_release_mm','max_support_after_lift_mm','native_range_violation_rad',
        'support_escape_max_unapproved_overlap_mm','self_escape_max_unapproved_overlap_mm',
        'unmodified_joint_range_violation_rad','max_ring_self_after_release_mm')
    values=np.array([row[k] for k in keys])
    if not np.isfinite(values).all() or np.any(values<0):return False
    if (row['max_self_after_release_mm']>1e-6 or row['max_support_after_lift_mm']>.15
        or row['native_range_violation_rad']>(SOFT_JOINT_RESIDUAL_RAD if dynamic else 1e-6)
        or row.get('unmodified_joint_range_violation_rad',float('inf'))>1e-6
        or row['max_ring_self_after_release_mm']>1e-6
        or row['support_escape_max_unapproved_overlap_mm']>1e-6
        or row['self_escape_max_unapproved_overlap_mm']>1e-6
        or not row['support_escape_complete'] or not row['self_escape_complete']):return False
    cmc=row.get('native_cmc_after_release_min_ticks',float('nan'))
    if not np.isfinite(cmc) or cmc < cmc_native_min()-(1. if dynamic else 1e-6):return False
    if dynamic:return (row['finite'] and row['max_tracking_rad']<=INITIAL_CONTACT_TRACKING_RAD
        and row['max_tracking_after_clearance_rad']<=.12)
    return (row['max_goal_step_ticks']<=2 and row['support_escape_max_step_increase_mm']<=1e-6
        and row['self_escape_max_step_increase_mm']<=1e-6)


def main(request,output,*,plan_type=BookGraspPlan):
    began=time.perf_counter();out=Path(output)
    if out.exists():raise ValueError('Refuse to reuse an old output')
    plan=plan_type(request);raw=plan.source_position()
    candidate_rows=plan.plan(raw)
    if plan.CAPTURE_PLACEMENT or plan.CAPTURE_BEFORE_PHASE is not None:
        raise ValueError('EMPTY finish must have no PLACE checkpoint')
    if sum(p==TURN_PHASE for p,q in candidate_rows)!=len(motion_goals())-1:
        raise ValueError('Incomplete EMPTY reference motion')
    bound_hashes=getattr(plan,'proof_hashes',extra_hashes)
    before=code_hashes();extra_before=bound_hashes();paths=[]
    for off in offsets():
        start=raw+off;rows=plan.plan(start)
        samples=np.vstack([start]+[q for _,q in rows])
        paths.append(dict(offset=off,rows=rows,angles=ticks_to_sim(samples),
            lift_count=sum(p==LIFT for p,_ in rows),
            tip_count=sum(p==THUMB_RELEASE for p,_ in rows),
             ring_count=sum(p==RING_RELEASE for p,_ in rows),
            max_step=int(np.max(abs(np.diff(samples,axis=0)),initial=0))))
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg);static=[];dynamic=[];manifests=[];raw_ring_observations=[];photo_resolution_observations=[]
    for front in FRONTS:
        m,manifest=build_cad(env,out.with_name('reverse_empty_CAD_scene.xml'),book_projection_mm=front)
        manifest['observed_model_correction']=correct_model(m,raw);manifests.append(manifest)
        if derive_scope(m)!=load_scope():raise ValueError('Native support-contact scope changed')
        m.geom_contype[m.geom('cube').id]=0;m.geom_conaffinity[m.geom('cube').id]=0
        hand={m.body('palm').id}
        for b in range(m.nbody):
            if int(m.body_parentid[b]) in hand:hand.add(b)
        support={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
        inspect_raw=cached_contacts(m,hand,support)
        raw_ring_max=[0.];photo_discarded={}
        photo_resolution_observations.append(photo_discarded)
        raw_ring_observations.append(raw_ring_max)
        def inspect(d,initial_release=False):
            support_pairs,self_pairs=inspect_raw(d)
            raw_ring_max[0]=max(raw_ring_max[0],self_pairs.get(CONTACT_PAIR,0.))
            classified=classify_self_contacts(self_pairs)
            resolved=classify_photo_initial(classified,initial_release=initial_release)
            for pair,depth in classified.items():
                if pair not in resolved:photo_discarded[pair]=max(photo_discarded.get(pair,0.),depth)
            return support_pairs,resolved
        joints=m.actuator_trnid[:,0]
        lower=np.maximum(m.jnt_range[joints,0],m.actuator_ctrlrange[:,0])
        upper=np.minimum(m.jnt_range[joints,1],m.actuator_ctrlrange[:,1])
        def pose(d,angles):
            d.qpos[:16]=angles;d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0]
            mujoco.mj_forward(m,d)
        def excess(angles):
            return float(max(0.,np.max(lower-angles),np.max(angles-upper)))
        unchanged_joints=np.arange(16)!=12
        def unchanged_excess(angles):
            return float(max(0.,np.max((lower-angles)[...,unchanged_joints]),
                             np.max((angles-upper)[...,unchanged_joints])))
        for path in paths:
            rows=path['rows'];angles=path['angles'];d=mujoco.MjData(m);pose(d,angles[0])
            pairs,self_pairs=inspect(d,initial_release=True)
            if self_pairs.get(CASING_PAIR,0.)>CASING_INITIAL_LIMIT_MM or self_pairs.get(EXTRA_RING_PAIR,0.)>EXTRA_RING_INITIAL_LIMIT_MM or self_pairs.get(THUMB_NEIGHBOR_PAIR,0.)>THUMB_NEIGHBOR_LIMIT_MM:
                raise ValueError('OFF initial casing/ring contact exceeds its separately recorded model bound')
            releases=path['ring_count']+path['tip_count']
            deadline=releases+min(path['lift_count'],SUPPORT_ROOT_ESCAPE_GOALS)
            escape=ContactEscape(pairs,ALLOWED_SUPPORT_PAIRS,MAX_INITIAL_SUPPORT_MM,deadline,monotonic=True)
            self_deadline=path['ring_count']+min(path['tip_count'],THUMB_ESCAPE_GOALS)
            self_escape=ContactEscape(self_pairs,SELF_PAIRS,MAX_INITIAL_THUMB_SELF_MM,self_deadline,monotonic=True)
            max_self=max(self_pairs.values(),default=0.);max_range=excess(angles);after=self_after=ring_after=0.;cmc_after=float("inf")
            ring_clear_at=None if any(p in self_pairs for p in RING_SELF_PAIRS) else 0
            for index,((phase,q),goal) in enumerate(zip(rows,angles[1:]),1):
                pose(d,goal);pairs,self_pairs=inspect(d,initial_release=phase==RING_RELEASE);escape.observe(pairs,index);self_escape.observe(self_pairs,index)
                sm=max(self_pairs.values(),default=0.);max_self=max(max_self,sm)
                ring_sm=max((self_pairs.get(p,0.) for p in RING_SELF_PAIRS),default=0.)
                if not ring_sm and ring_clear_at is None:ring_clear_at=index
                if phase!=RING_RELEASE:
                    ring_after=max(ring_after,ring_sm);cmc_after=min(cmc_after,float(goal[12]*4096/(2*np.pi)+2048))
                if phase not in (RING_RELEASE,THUMB_RELEASE):self_after=max(self_after,sm)
                if phase not in (LIFT,RING_RELEASE,THUMB_RELEASE):after=max(after,max(pairs.values(),default=0.))
            static.append(dict(book_front_mm=front,offset_ticks=path['offset'].tolist(),
                max_self_overlap_mm=max_self,max_support_after_lift_mm=after,
                max_self_after_release_mm=self_after,native_cmc_after_release_min_ticks=cmc_after,
                max_goal_step_ticks=path['max_step'],native_range_violation_rad=max_range,
                unmodified_joint_range_violation_rad=unchanged_excess(angles),
                max_ring_self_after_release_mm=ring_after,
                ring_clear_goal_ticks=clear_goal(raw+path['offset'],rows,ring_clear_at,14),
                ring_clear_complete_at=ring_clear_at,thumb_tip_clear_goal_ticks=clear_goal(raw+path['offset'],rows,self_escape.complete_at,0),
                thumb_clear_goal_ticks=clear_goal(raw+path['offset'],rows,self_escape.complete_at,3),
                support_escape_deadline=deadline,self_escape_deadline=self_deadline,
                **escape_fields(escape),**self_escape.result()))
        print(f'Book front {front:g}mm: 67 complete preparation/grasp paths checked',flush=True)
        support_envelope={}
        for case in static:
            if case['book_front_mm']==front:
                for pair,depth in case['initial_support_contacts_mm'].items():
                    support_envelope[pair]=max(support_envelope.get(pair,0.),depth)
        self_envelope={}
        for case in static:
            if case['book_front_mm']==front:
                for pair,depth in case['initial_self_contacts_mm'].items():self_envelope[pair]=max(self_envelope.get(pair,0.),depth)
        rows=paths[0]['rows'];angles=paths[0]['angles'];lift_s=paths[0]['lift_count']*.05
        dynamics=[('initial_hold',angles[0])]*10
        inserted=False
        for (phase,q),goal in zip(rows,angles[1:]):
            if phase==TURN_PHASE and not inserted:inserted=True
            dynamics.append((phase,goal))
        dynamics.extend([('final_hold',angles[-1])]*20)
        gains=m.actuator_gainprm.copy();bias=m.actuator_biasprm.copy()
        for scale in SCALES:
            m.actuator_gainprm[:]=gains;m.actuator_biasprm[:]=bias
            m.actuator_gainprm[:,0]*=scale;m.actuator_biasprm[:,1]*=scale
            d=mujoco.MjData(m);pose(d,angles[0]);d.ctrl[:]=angles[0]
            tip_s=paths[0]['tip_count']*.05
            ring_s=paths[0]['ring_count']*.05
            pairs,self_pairs=inspect(d,initial_release=True)
            if self_pairs.get(CASING_PAIR,0.)>CASING_INITIAL_LIMIT_MM or self_pairs.get(EXTRA_RING_PAIR,0.)>EXTRA_RING_INITIAL_LIMIT_MM or self_pairs.get(THUMB_NEIGHBOR_PAIR,0.)>THUMB_NEIGHBOR_LIMIT_MM:
                raise ValueError('OFF initial casing/ring contact exceeds its separately recorded model bound')
            deadline=.5+ring_s+tip_s+min(lift_s,ROOT_ESCAPE_SECONDS)
            nominal_initial_support=dict(pairs)
            initial_bound=dict(support_envelope);hold_contacts=dict(pairs)
            probe=mujoco.MjData(m);pose(probe,angles[0]);probe.ctrl[:]=angles[0]
            for _ in range(round(.5/m.opt.timestep)):
                mujoco.mj_step(m,probe);probe_pairs,_=inspect(probe)
                for pair,depth in probe_pairs.items():hold_contacts[pair]=max(hold_contacts.get(pair,0.),depth)
            for pair,depth in hold_contacts.items():
                if pair not in nominal_initial_support:
                    if depth>.15:raise ValueError('New passive enable support contact exceeds the existing 0.15mm model allowance')
                    initial_bound[pair]=max(initial_bound.get(pair,0.),depth)
            hold_evidence=dict(duration_seconds=.5,goal_ticks=raw.tolist(),contact_maximums_mm=hold_contacts,new_pair_depth_limit_mm=.15)
            escape=ContactEscape(initial_bound,ALLOWED_SUPPORT_PAIRS,MAX_INITIAL_SUPPORT_MM,deadline,monotonic=False)
            self_deadline=.5+ring_s+tip_s
            nominal_initial_self=dict(self_pairs)
            self_escape=ContactEscape(self_envelope,SELF_PAIRS,MAX_INITIAL_THUMB_SELF_MM,self_deadline,monotonic=False)
            max_self=max(self_pairs.values(),default=0.)
            tracking=tracking_after=max_range=unchanged_range=after=self_after=ring_after=elapsed=0.;finite=True;cmc_after=float("inf")
            for phase,goal in dynamics:
                d.ctrl[:]=goal
                for _ in range(round(.05/m.opt.timestep)):
                    mujoco.mj_step(m,d);elapsed+=m.opt.timestep
                    finite=bool(np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all())
                    if not finite:break
                    pairs,self_pairs=inspect(d,initial_release=phase in ('initial_hold',RING_RELEASE));escape.observe(pairs,elapsed);self_escape.observe(self_pairs,elapsed)
                    sm=max(self_pairs.values(),default=0.)
                    max_self=max(max_self,sm);tracking=max(tracking,float(max(abs(d.qpos[:16]-goal))))
                    max_range=max(max_range,excess(d.qpos[:16]))
                    unchanged_range=max(unchanged_range,unchanged_excess(d.qpos[:16]))
                    if phase not in ('initial_hold',RING_RELEASE):
                        cmc_after=min(cmc_after,float(d.qpos[12]*4096/(2*np.pi)+2048))
                        ring_after=max(ring_after,max((self_pairs.get(p,0.) for p in RING_SELF_PAIRS),default=0.))
                    if phase not in ('initial_hold',RING_RELEASE,THUMB_RELEASE):self_after=max(self_after,sm)
                    if phase not in ('initial_hold',LIFT,RING_RELEASE,THUMB_RELEASE):
                        after=max(after,max(pairs.values(),default=0.))
                        tracking_after=max(tracking_after,float(max(abs(d.qpos[:16]-goal))))
                if not finite:break
            row=dict(book_front_mm=front,gain_scale=scale,finite=finite,max_tracking_rad=tracking,
                nominal_initial_support_contacts_mm=nominal_initial_support,nominal_initial_self_contacts_mm=nominal_initial_self,passive_enable_hold_support=hold_evidence,
                max_self_overlap_mm=max_self,max_support_after_lift_mm=after,
                max_self_after_release_mm=self_after,native_cmc_after_release_min_ticks=cmc_after,max_tracking_after_clearance_rad=tracking_after,
                native_range_violation_rad=max_range,support_escape_deadline=deadline,
                unmodified_joint_range_violation_rad=unchanged_range,
                max_ring_self_after_release_mm=ring_after,
                self_escape_deadline=self_deadline,**escape_fields(escape),**self_escape.result())
            dynamic.append(row)
            print(json.dumps(dict(front=front,gain=scale,passed=passed(row,True),tracking=tracking)),flush=True)
    after_hash=code_hashes();extra_after=bound_hashes()
    unchanged=before==after_hash;extra_unchanged=extra_before==extra_after
    accepted=unchanged and extra_unchanged and len(static)==201 and len(dynamic)==9 and all(passed(s) for s in static) and all(passed(s,True) for s in dynamic)
    r=dict(plan_version=plan.PLAN_VERSION,source_sha256=plan.SOURCE_SHA,plan_sha256=plan.plan_digest(raw),
        start_ticks=raw.tolist(),last_goal_ticks=plan.plan(raw)[-1][1].tolist(),mapping=list(SIM_TO_MOTOR_IDS),
        static_screens=static,dynamic_sensitivity=dynamic,code_hashes=after_hash,code_unchanged=unchanged,
        extra_code_hashes=extra_after,extra_code_unchanged=extra_unchanged,model_manifests=manifests,
        photo_initial_thumb_contact_resolution=photo_resolution_metadata(),initial_resolution_discarded_maximums_by_front=photo_resolution_observations,initial_resolution_forces_preserved=True,native_ranges_unchanged=not policy(raw)['model_range_extended'],observed_model_policy=policy(raw),hardware_commands_sent=0,
        passive_support_reentry_policy=support_reentry_policy(),
        max_ring_tip_palm_raw_overlap_mm=max(v[0] for v in raw_ring_observations),
        accepted_for_supervised_empty_preparation=accepted,accepted_for_supervised_grasp_approach=False,
        empty_hand_only=True,placement_capture_only=False,joint_goals_fixed_after_placement=False,full_quarter_reference_path=True,full_empty_return_and_finish_hand_route_checked=True,
        casing_release_motor_id=4,initial_casing_contact_limit_mm=CASING_INITIAL_LIMIT_MM,
        initial_support_scope=INITIAL_SUPPORT_SCOPE,no_contact_outside_reviewed_initial_envelope=True,interrupted_suffix_verified=False,experimental_loaded_trial=False,empty_motion_basis=empty_basis(),initial_ring_neighbor_limit_mm=EXTRA_RING_INITIAL_LIMIT_MM,initial_thumb_neighbor_limit_mm=THUMB_NEIGHBOR_LIMIT_MM,initial_self_scope=INITIAL_SELF_SCOPE,
        full_CAD_tip_contacts=True,cube_disabled_entire_screen=True,
        cube_contact_verified=False,placement_checkpoint_required=False,
        rotation_included=bool(getattr(plan,'ROTATION_INCLUDED',False)),
        native_placement_hold_samples=0,passive_ID4_monotonic_reentry=True,combined_motion_time_limit_seconds=plan.ON_TIMEOUT,physical_calibration_verified=False,free_space_certificate=False,
        initial_thumb_contact_operator_observed=False,
        initial_self_depth_limit_mm=MAX_INITIAL_THUMB_SELF_MM,
        initial_support_depth_limit_mm=MAX_INITIAL_SUPPORT_MM,
        initial_contact_model_tracking_limit_rad=INITIAL_CONTACT_TRACKING_RAD,
        actual_tracking_stop_rad=.18,
        book_front_is_photo_estimate=True,elapsed_seconds=time.perf_counter()-began,
        empty_root_finish_goal_count=sum(p==FINISH_PHASE for p,q in candidate_rows),
        note='EMPTY reference plus bounded root finish. Scoped observed passive ID4 recovery and 0.1mm ring-tip/palm-box classification tolerance; forces preserved. Full hand route only. Complete EMPTY return, reference and root finish. Cube absent. No physical rotation claim. Explicit start and final holds; no live policy. Full CAD fingertip and middle/ring housing checks.')
    out.write_text(json.dumps(r,indent=2),encoding='utf-8')
    print('Supervised book grasp hand-route screen:',accepted,flush=True)
    return 0 if accepted else 2


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('request');parser.add_argument('output')
    parser.add_argument('--diagnostic-single',action='store_true')
    args=parser.parse_args()
    if args.diagnostic_single:
        FRONTS=(-10.,);SCALES=(1.,);offsets=lambda:[np.zeros(16,dtype=int)]
    raise SystemExit(main(args.request,args.output))
