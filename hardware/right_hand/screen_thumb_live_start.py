"""WSL MuJoCo screen of a specific fresh OFF request. Never imports motor code."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from hand_motor_mapping import ticks_to_sim, SIM_TO_MOTOR_IDS
from thumb_live_start_plan import CurrentPlan
from hand_preparation_plan import PreparationPlan
from prepared_placement_plan import PlacementPlan
from grasp_approach_plan import GraspApproachPlan
from grasp_place_plan import GraspPlacePlan
from cube_small_rotation_plan import SmallRotationPlan
from supported_yaw_empty_plan import SupportedYawEmptyPlan
from supported_cube_rotation_plan import SupportedCubeRotationPlan
from slow_grasp_demo_plan import SlowGraspDemoPlan
from grasped_yaw_plan import GraspedYawPlan
from initial_contact_escape import ContactEscape


def main(request,output):
    version=json.loads(Path(request).read_bytes()).get('plan_version')
    kind={CurrentPlan.PLAN_VERSION:CurrentPlan,PreparationPlan.PLAN_VERSION:PreparationPlan,
          PlacementPlan.PLAN_VERSION:PlacementPlan,GraspApproachPlan.PLAN_VERSION:GraspApproachPlan,
          GraspPlacePlan.PLAN_VERSION:GraspPlacePlan,
          SmallRotationPlan.PLAN_VERSION:SmallRotationPlan,
          SupportedYawEmptyPlan.PLAN_VERSION:SupportedYawEmptyPlan,
          SupportedCubeRotationPlan.PLAN_VERSION:SupportedCubeRotationPlan,
          SlowGraspDemoPlan.PLAN_VERSION:SlowGraspDemoPlan,
          GraspedYawPlan.PLAN_VERSION:GraspedYawPlan}.get(version)
    if kind is None:raise ValueError('Unsupported current-pose plan')
    plan=kind(request);raw=plan.source_position()
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build_with_middle_thumb_hulls(env);d=mujoco.MjData(m)
    cube=m.geom('cube').id;m.geom_contype[cube]=0;m.geom_conaffinity[cube]=0
    palm=m.body('palm').id;hand={palm}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_glass','support_wood')}
    def overlaps():
        self_depth=support_depth=0.
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2)
            ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
            if ha and hb:self_depth=max(self_depth,-float(c.dist)*1000)
            if (a in support and hb) or (b in support and ha):support_depth=max(support_depth,-float(c.dist)*1000)
        return self_depth,support_depth
    def self_contacts():
        pairs={}
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2)
            if int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand and c.dist<0:
                pair='|'.join(sorted((m.geom(a).name,m.geom(b).name)))
                pairs[pair]=max(pairs.get(pair,0.),-float(c.dist)*1000)
        return pairs
    def pose(q):
        d.qpos[:16]=ticks_to_sim(q);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0]
        mujoco.mj_forward(m,d)
    def range_excess(q):
        angles=ticks_to_sim(q);excess=np.zeros(16)
        for index,joint in enumerate(m.actuator_trnid[:,0]):
            if m.jnt_limited[joint]:
                lo,hi=m.jnt_range[joint]
                excess[index]=max(excess[index],float(lo-angles[index]),float(angles[index]-hi))
            if m.actuator_ctrllimited[index]:
                lo,hi=m.actuator_ctrlrange[index]
                excess[index]=max(excess[index],float(lo-angles[index]),float(angles[index]-hi))
        return excess
    offsets=[np.zeros(16,dtype=int),np.full(16,-4),np.full(16,4)]
    for i in range(16):
        for sign in (-1,1):
            v=np.zeros(16,dtype=int);v[i]=4*sign;offsets.append(v)
    offsets.extend(np.random.default_rng(20261021).integers(-4,5,(32,16)))
    screens=[]
    for off in offsets:
        start=raw+off;rows=plan.plan(start);pose(start)
        sd,initial_support=overlaps();max_self=sd;max_step=0;increase=0.;thumb_support=0.
        escape=None
        if hasattr(plan,'INITIAL_SELF_ESCAPE_PAIRS'):
            escape=ContactEscape(self_contacts(),plan.INITIAL_SELF_ESCAPE_PAIRS,
                plan.MAX_INITIAL_SELF_ESCAPE_MM,plan.INITIAL_SELF_ESCAPE_GOALS,monotonic=True)
        clear_pip=(int(start[14]) if escape is not None and not escape.initial else None)
        previous_support=initial_support;last=start.copy()
        initial_excess=range_excess(start);previous_excess=initial_excess.copy()
        native_range_violation=float(max(initial_excess));unapproved_violation=0.
        reentry_phase=getattr(plan,'NATIVE_REENTRY_PHASE',None)
        reentry_bound=getattr(plan,'MAX_START_RANGE_EXCESS_RAD',0.)
        reentry_monotonic=True;reentry_complete=True
        if max(initial_excess)>reentry_bound+1e-6:unapproved_violation=float(max(initial_excess))
        for goal_index,(phase,q) in enumerate(rows,1):
            max_step=max(max_step,int(max(abs(q-last))));last=q.copy()
            pose(q);s,w=overlaps();max_self=max(max_self,s)
            if escape is not None:
                escape.observe(self_contacts(),goal_index)
                if clear_pip is None and escape.complete_at is not None:clear_pip=int(q[14])
            increase=max(increase,w-previous_support);previous_support=w
            if phase=='stage_1_thumb_prefix':thumb_support=max(thumb_support,w)
            excess=range_excess(q);native_range_violation=max(native_range_violation,float(max(excess)))
            if reentry_phase and phase==reentry_phase:
                if np.any(excess>previous_excess+1e-6):
                    reentry_monotonic=False;unapproved_violation=max(unapproved_violation,float(max(excess)))
            elif max(excess)>1e-6:
                reentry_complete=False;unapproved_violation=max(unapproved_violation,float(max(excess)))
            previous_excess=excess
        row=dict(offset_ticks=off.tolist(),max_self_overlap_mm=max_self,
            initial_support_overlap_mm=initial_support,max_support_increase_mm=increase,
            thumb_support_overlap_mm=thumb_support,max_goal_step_ticks=max_step,
            native_range_violation_rad=native_range_violation,
            initial_native_range_excess_rad=float(max(initial_excess)),
            unapproved_native_range_violation_rad=unapproved_violation,
            native_reentry_phase=reentry_phase,native_reentry_monotonic=reentry_monotonic,
            native_reentry_complete=reentry_complete)
        if escape is not None:row.update(escape.result())
        if hasattr(plan,'CONTACT_RELEASE_BEFORE_PHASE'):
            row.update(contact_clear_pip_goal_ticks=clear_pip,
                contact_clear_pip_travel_ticks=None if clear_pip is None else int(start[14])-clear_pip)
        screens.append(row)
    print('Fresh-start static screen complete',flush=True)
    # Native model only. No temporary widening of a joint or actuator range.
    native_gain=m.actuator_gainprm.copy();native_bias=m.actuator_biasprm.copy()
    screen_rows=(plan.screen_dynamic_path(raw) if hasattr(plan,'screen_dynamic_path') else plan.plan(raw))
    dynamic=[];path=[('initial_hold',raw)]*10+screen_rows+[('final_hold',plan.plan(raw)[-1][1])]*20
    for gain in (.5,1.,1.5):
        m.actuator_gainprm[:]=native_gain;m.actuator_biasprm[:]=native_bias
        m.actuator_gainprm[:,0]*=gain;m.actuator_biasprm[:,1]*=gain
        d=mujoco.MjData(m);pose(raw);d.ctrl[:]=ticks_to_sim(raw)
        escape=None
        if hasattr(plan,'INITIAL_SELF_ESCAPE_PAIRS'):
            escape=ContactEscape(self_contacts(),plan.INITIAL_SELF_ESCAPE_PAIRS,
                plan.MAX_INITIAL_SELF_ESCAPE_MM,plan.INITIAL_SELF_ESCAPE_DYNAMIC_S,monotonic=False)
        max_tracking=max_self=max_support=0.
        elapsed=0.
        for phase,q in path:
            goal=ticks_to_sim(q);d.ctrl[:]=goal
            for _ in range(round(.05/m.opt.timestep)):
                mujoco.mj_step(m,d);s,w=overlaps()
                elapsed+=m.opt.timestep
                if escape is not None:escape.observe(self_contacts(),elapsed)
                max_self=max(max_self,s);max_support=max(max_support,w)
                max_tracking=max(max_tracking,float(max(abs(d.qpos[:16]-goal))))
        row=dict(gain_scale=gain,max_tracking_rad=max_tracking,max_self_overlap_mm=max_self,
                 max_support_overlap_mm=max_support,finite=bool(np.isfinite(d.qpos).all()))
        if escape is not None:row.update(escape.result())
        dynamic.append(row);print(json.dumps(row),flush=True)
    def self_pass(s):
        if not hasattr(plan,'INITIAL_SELF_ESCAPE_PAIRS'):return s['max_self_overlap_mm']<=1e-6
        if 'contact_clear_pip_travel_ticks' in s:
            travel=s['contact_clear_pip_travel_ticks']
            if travel is None or not 0<=travel<=plan.MAX_CONTACT_RELEASE_TRAVEL_TICKS:return False
        return s['self_escape_max_unapproved_overlap_mm']<=1e-6 and s['self_escape_complete']
    static_pass=all(self_pass(s) and s['initial_support_overlap_mm']<=.15
        and s['max_support_increase_mm']<=1e-6 and s['thumb_support_overlap_mm']<=1e-6
        and s['max_goal_step_ticks']<=2 and s['unapproved_native_range_violation_rad']<=1e-6
        and s['native_reentry_monotonic'] and s['native_reentry_complete'] for s in screens)
    dynamic_pass=all(s['finite'] and s['max_tracking_rad']<=.12 and self_pass(s)
                     and s['max_support_overlap_mm']<=.15 for s in dynamic)
    report=dict(plan_version=plan.PLAN_VERSION,source_sha256=plan.SOURCE_SHA,
        plan_sha256=plan.plan_digest(raw),mapping=list(SIM_TO_MOTOR_IDS),
        start_ticks=raw.tolist(),last_goal_ticks=plan.plan(raw)[-1][1].tolist(),
        static_screens=screens,dynamic_sensitivity=dynamic,model_manifest=manifest,
        accepted=static_pass and dynamic_pass,native_ranges_unchanged=True,
        physical_calibration_verified=False,hardware_commands_sent=0,
        contact_release_travel_limit_ticks=getattr(plan,'MAX_CONTACT_RELEASE_TRAVEL_TICKS',None),
        planner_sha256=hashlib.sha256((Path(__file__).parent/'thumb_live_start_plan.py').read_bytes()).hexdigest(),
        extra_planner_sha256={name:hashlib.sha256((Path(__file__).parent/name).read_bytes()).hexdigest()
                              for name in getattr(plan,'EXTRA_PLANNER_FILES',())},
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    Path(output).write_text(json.dumps(report,indent=2),encoding='utf-8')
    return 0 if report['accepted'] else 2


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('request');parser.add_argument('output');args=parser.parse_args()
    raise SystemExit(main(args.request,args.output))
