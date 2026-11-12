"""Native, device-free replay of a fresh EMPTY book-support preparation.

The initial support contact is explicitly separated, never labelled free
space. New/deeper contacts, self collision, range changes, or failure to clear
before the other fingers move reject this experimental route.
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
from book_empty_preparation_plan import (BookPreparationPlan,FRONTS,LIFT,
    MAX_INITIAL_SUPPORT_MM,SUPPORT_ESCAPE_STATIC_GOALS,SUPPORT_ESCAPE_DYNAMIC_SECONDS,
    ALLOWED_SUPPORT_PAIRS,offsets,code_hashes)
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS
from initial_contact_escape import ContactEscape
from book_support_contact_scope import derive_scope,load_scope

def contact_metrics(m,d,hand,support):
    pairs={};self_depth=0.
    for c in d.contact:
        if c.dist>=0:continue
        a,b=int(c.geom1),int(c.geom2)
        ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
        depth=-float(c.dist)*1000
        if ha and hb:self_depth=max(self_depth,depth)
        if (a in support and hb) or (b in support and ha):
            pair='|'.join(sorted((m.geom(a).name,m.geom(b).name)))
            pairs[pair]=max(pairs.get(pair,0.),depth)
    return pairs,self_depth


def cached_contact_metrics(m,hand,support):
    """Same contact classification, with constant model lookups done once."""
    is_hand=np.isin(m.geom_bodyid,list(hand))
    is_support=np.zeros(m.ngeom,dtype=bool);is_support[list(support)]=True
    names=[m.geom(g).name for g in range(m.ngeom)]
    def inspect(d):
        if d.ncon==0:return {},0.
        indices=np.flatnonzero(d.contact.dist<0)
        if not len(indices):return {},0.
        a=d.contact.geom1[indices];b=d.contact.geom2[indices]
        depth=-d.contact.dist[indices]*1000
        ha,hb=is_hand[a],is_hand[b]
        self_depth=float(np.max(depth[ha&hb],initial=0.))
        selected=np.flatnonzero((is_support[a]&hb)|(is_support[b]&ha));pairs={}
        for i in selected:
            pair='|'.join(sorted((names[a[i]],names[b[i]])))
            pairs[pair]=max(pairs.get(pair,0.),float(depth[i]))
        return pairs,self_depth
    return inspect


def escape_fields(escape):
    # Keep these support fields separate from historical self-contact proofs.
    return {k.replace('initial_self_contacts','initial_support_contacts').replace('self_escape','support_escape'):v
            for k,v in escape.result().items()}


def passed(row,dynamic=False):
    numbers=[row[k] for k in ('max_self_overlap_mm','max_support_after_lift_mm',
        'support_escape_max_unapproved_overlap_mm','native_range_violation_rad')]
    if not np.isfinite(numbers).all() or min(numbers)<0:return False
    if (row['max_self_overlap_mm']>1e-6 or row['max_support_after_lift_mm']>.15
            or row['support_escape_max_unapproved_overlap_mm']>1e-6
            or row['native_range_violation_rad']>1e-6 or not row['support_escape_complete']):return False
    if dynamic:return row['finite'] and 0<=row['max_tracking_rad']<=.12
    return row['max_goal_step_ticks']<=2 and row['support_escape_max_step_increase_mm']<=1e-6


def main(request,output):
    began=time.perf_counter();timings=[]
    plan=BookPreparationPlan(request);raw=plan.source_position();out=Path(output)
    if out.exists():raise ValueError('Refuse to reuse an old output')
    before_hashes=code_hashes()
    # The same 67 exact command paths apply to all three support hypotheses.
    # Cache all samples/conversions without dropping any model replay.
    paths=[]
    for off in offsets():
        start=raw+off;rows=plan.plan(start)
        samples=np.vstack([start]+[q for _,q in rows])
        paths.append(dict(offset=off,rows=rows,angles=ticks_to_sim(samples),
            lift_count=sum(p==LIFT for p,_ in rows),
            max_step=int(np.max(abs(np.diff(samples,axis=0)),initial=0))))
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg);static=[];dynamic=[];manifests=[]
    for front in FRONTS:
        front_began=time.perf_counter()
        m,manifest=build_book(env,out.with_name('book_screen_scene.xml'),book_projection_mm=front,
            book_ylo_mm=-500.,book_yhi_mm=500.,support_projection_mm=50.)
        manifests.append(manifest)
        if derive_scope(m)!=load_scope():
            raise ValueError('Native root/body support-contact scope differs from the reviewed table')
        m.geom_contype[m.geom('cube').id]=0;m.geom_conaffinity[m.geom('cube').id]=0
        hand={m.body('palm').id}
        for b in range(m.nbody):
            if int(m.body_parentid[b]) in hand:hand.add(b)
        support={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
        inspect_contacts=cached_contact_metrics(m,hand,support)
        joints=m.actuator_trnid[:,0]
        lower=np.maximum(m.jnt_range[joints,0],m.actuator_ctrlrange[:,0])
        upper=np.minimum(m.jnt_range[joints,1],m.actuator_ctrlrange[:,1])
        def pose(d,angles):
            d.qpos[:16]=angles;d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0]
            mujoco.mj_forward(m,d)
        def excess(angles):
            return float(max(0.,np.max(lower-angles),np.max(angles-upper)))
        for path in paths:
            off=path['offset'];rows=path['rows'];angles=path['angles'];lift_count=path['lift_count']
            d=mujoco.MjData(m);pose(d,angles[0]);pairs,sm=inspect_contacts(d)
            deadline=min(lift_count,SUPPORT_ESCAPE_STATIC_GOALS)
            escape=ContactEscape(pairs,ALLOWED_SUPPORT_PAIRS,MAX_INITIAL_SUPPORT_MM,deadline,monotonic=True)
            max_self=sm;max_range=excess(angles);after=0.;step=path['max_step']
            for index,((phase,q),goal) in enumerate(zip(rows,angles[1:]),1):
                pose(d,goal)
                pairs,sm=inspect_contacts(d);escape.observe(pairs,index)
                max_self=max(max_self,sm)
                if phase!=LIFT:after=max(after,max(pairs.values(),default=0.))
            static.append(dict(book_front_mm=front,offset_ticks=off.tolist(),
                max_self_overlap_mm=max_self,max_support_after_lift_mm=after,
                max_goal_step_ticks=step,native_range_violation_rad=max_range,
                support_escape_deadline=deadline,**escape_fields(escape)))
        print(f'Book front {front:g}mm: 67 native command paths checked',flush=True)
        static_finished=time.perf_counter()
        gains=m.actuator_gainprm.copy();bias=m.actuator_biasprm.copy();rows=plan.plan(raw)
        lift_s=sum(p==LIFT for p,_ in rows)*.05
        dynamic_path=[('initial_hold',ticks_to_sim(raw))]*10+[(p,a) for (p,_),a in zip(rows,paths[0]['angles'][1:])]+[('final_hold',paths[0]['angles'][-1])]*20
        for scale in (.5,1.,1.5):
            m.actuator_gainprm[:]=gains;m.actuator_biasprm[:]=bias
            m.actuator_gainprm[:,0]*=scale;m.actuator_biasprm[:,1]*=scale
            d=mujoco.MjData(m);pose(d,paths[0]['angles'][0]);d.ctrl[:]=paths[0]['angles'][0]
            pairs,sm=inspect_contacts(d)
            deadline=min(.5+lift_s,SUPPORT_ESCAPE_DYNAMIC_SECONDS)
            escape=ContactEscape(pairs,ALLOWED_SUPPORT_PAIRS,MAX_INITIAL_SUPPORT_MM,deadline,monotonic=False)
            max_self=sm;max_tracking=max_range=after=0.;elapsed=0.;finite=True
            for phase,goal in dynamic_path:
                d.ctrl[:]=goal
                for _ in range(round(.05/m.opt.timestep)):
                    mujoco.mj_step(m,d);elapsed+=m.opt.timestep
                    finite=bool(np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all())
                    if not finite:break
                    pairs,sm=inspect_contacts(d);escape.observe(pairs,elapsed)
                    max_self=max(max_self,sm);max_tracking=max(max_tracking,float(max(abs(d.qpos[:16]-goal))))
                    # Native integration may approach a joint limit; no widening.
                    max_range=max(max_range,excess(d.qpos[:16]))
                    if phase not in ('initial_hold',LIFT):after=max(after,max(pairs.values(),default=0.))
                if not finite:break
            row=dict(book_front_mm=front,gain_scale=scale,finite=finite,max_tracking_rad=max_tracking,
                max_self_overlap_mm=max_self,max_support_after_lift_mm=after,
                native_range_violation_rad=max_range,support_escape_deadline=deadline,**escape_fields(escape))
            dynamic.append(row)
            print(json.dumps(dict(front=front,gain=scale,passed=passed(row,True),
                tracking=max_tracking,unapproved=row['support_escape_max_unapproved_overlap_mm'])),flush=True)
        timings.append(dict(book_front_mm=front,build_and_static_seconds=static_finished-front_began,
            dynamics_seconds=time.perf_counter()-static_finished))
    after_hashes=code_hashes()
    accepted=before_hashes==after_hashes and all(passed(s) for s in static) and all(passed(s,True) for s in dynamic)
    r=dict(plan_version=plan.PLAN_VERSION,source_sha256=plan.SOURCE_SHA,plan_sha256=plan.plan_digest(raw),
        start_ticks=raw.tolist(),last_goal_ticks=plan.plan(raw)[-1][1].tolist(),mapping=list(SIM_TO_MOTOR_IDS),
        static_screens=static,dynamic_sensitivity=dynamic,code_hashes=after_hashes,code_unchanged=before_hashes==after_hashes,
        model_manifests=manifests,native_ranges_unchanged=True,hardware_commands_sent=0,
        accepted_for_supervised_empty_preparation=accepted,physical_calibration_verified=False,
        free_space_certificate=False,book_front_is_photo_estimate=True,
        elapsed_seconds=time.perf_counter()-began,stage_timings=timings,
        note='Dedicated initial support separation, bounded per existing pair; no new/deeper contact. Geometry is approximate. Not a cube rotation trial.')
    out.write_text(json.dumps(r,indent=2),encoding='utf-8')
    print('Supervised EMPTY preparation screen:',accepted,flush=True)
    return 0 if accepted else 2


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('request');parser.add_argument('output')
    args=parser.parse_args();raise SystemExit(main(args.request,args.output))
