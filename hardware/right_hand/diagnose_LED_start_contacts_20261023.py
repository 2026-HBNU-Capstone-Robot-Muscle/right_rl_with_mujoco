"""Offline contact localization along a saved path; no camera/serial APIs."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from grasped_yaw_plan import GraspedYawPlan
from hand_motor_mapping import ticks_to_sim
from grasp_recovery_plan import interpolate
from slow_grasp_demo_plan import demo_opening
from initial_contact_escape import ContactEscape

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'right_hand_rotation_20261023/grasped_yaw_LED_complete_fixture'


def main():
    plan=GraspedYawPlan(FOLDER/'request.json');raw=plan.source_position()
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg)
    model,manifest=build_with_middle_thumb_hulls(env);data=mujoco.MjData(model)
    cube=model.geom('cube').id;model.geom_contype[cube]=0;model.geom_conaffinity[cube]=0
    palm=model.body('palm').id;hand={palm}
    for b in range(model.nbody):
        if int(model.body_parentid[b]) in hand:hand.add(b)
    def contacts(q):
        data.qpos[:16]=ticks_to_sim(q);data.qpos[-7:-4]=[0,0,2];data.qpos[-4:]=[1,0,0,0]
        mujoco.mj_forward(model,data);pairs={}
        for c in data.contact:
            a,b=int(c.geom1),int(c.geom2)
            if int(model.geom_bodyid[a]) in hand and int(model.geom_bodyid[b]) in hand and c.dist<0:
                key='|'.join(sorted((model.geom(a).name,model.geom(b).name)))
                pairs[key]=max(pairs.get(key,0.),-float(c.dist)*1000)
        return pairs
    initial=contacts(raw);previous=initial.copy();events=[];worst={}
    for index,(phase,q) in enumerate(plan.plan(raw),1):
        pairs=contacts(q)
        for pair,depth in pairs.items():
            if pair not in initial or depth>initial[pair]+1e-6 or depth>previous.get(pair,0.)+1e-6:
                event=dict(index=index,phase=phase,pair=pair,depth_mm=depth,
                    initial_mm=initial.get(pair,0.),previous_mm=previous.get(pair,0.),goal_ticks=q.tolist())
                if len(events)<6:events.append(event)
                if pair not in worst or depth>worst[pair]['depth_mm']:worst[pair]=event
        previous=pairs
    result=dict(start=raw.tolist(),initial_contacts=initial,first_events=events,worst_by_pair=worst)
    (FOLDER/'contact_diagnosis.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))
    scans=[]
    for clearance in (128,160,192,224,256,320,384):
        original=demo_opening(raw,plan._opened)
        prefix=[(p,q.copy()) for p,q in original if p=='stage_0_empty_recover']
        a=prefix[-1][1].copy();thumb=a.copy();delta=plan._opened[:4]-a[:4]
        alpha=min(1.,clearance/max(1,int(max(abs(delta)))))
        thumb[:4]=np.rint(a[:4]+delta*alpha).astype(int)
        a=interpolate(prefix,a,thumb,'stage_1_thumb_clearance')
        interpolate(prefix,a,plan._opened,'stage_2_empty_open')
        escape=ContactEscape(contacts(raw),plan.INITIAL_SELF_ESCAPE_PAIRS,
            plan.MAX_INITIAL_SELF_ESCAPE_MM,plan.INITIAL_SELF_ESCAPE_GOALS,monotonic=True)
        bad={}
        for index,(phase,q) in enumerate(prefix,1):
            pairs=contacts(q);escape.observe(pairs,index)
            for pair,depth in pairs.items():
                if pair not in initial:bad[pair]=max(bad.get(pair,0.),depth)
        row=dict(clearance_ticks=clearance,prefix_goals=len(prefix),new_contacts_mm=bad,**escape.result())
        scans.append(row);print('CANDIDATE',json.dumps(row),flush=True)
    (FOLDER/'thumb_clearance_candidates.json').write_text(json.dumps(scans,indent=2),encoding='utf-8')
    alternatives=[]
    for kind in ('delay_ring_tip_return','ring_rotation_first','whole_ring_first'):
        original=demo_opening(raw,plan._opened)
        if kind=='delay_ring_tip_return':
            prefix=[];tip=int(next(q[12] for p,q in reversed(original) if p=='stage_0_empty_recover'))
            for phase,q in original:
                q=q.copy()
                if phase=='stage_2_empty_open':q[12]=tip
                prefix.append((phase,q))
            interpolate(prefix,prefix[-1][1],plan._opened,'stage_2_empty_open')
        else:
            prefix=[(p,q.copy()) for p,q in original if p=='stage_0_empty_recover']
            a=prefix[-1][1].copy();ring=a.copy()
            motors=(13,) if kind=='ring_rotation_first' else (12,13,14,15)
            ring[list(motors)]=plan._opened[list(motors)]
            a=interpolate(prefix,a,ring,'stage_1_ring_clearance')
            thumb=a.copy();delta=plan._opened[:4]-a[:4]
            alpha=min(1.,128/max(1,int(max(abs(delta)))))
            thumb[:4]=np.rint(a[:4]+delta*alpha).astype(int)
            a=interpolate(prefix,a,thumb,'stage_1_thumb_clearance')
            interpolate(prefix,a,plan._opened,'stage_2_empty_open')
        escape=ContactEscape(contacts(raw),plan.INITIAL_SELF_ESCAPE_PAIRS,
            plan.MAX_INITIAL_SELF_ESCAPE_MM,plan.INITIAL_SELF_ESCAPE_GOALS,monotonic=True)
        bad={}
        for index,(phase,q) in enumerate(prefix,1):
            pairs=contacts(q);escape.observe(pairs,index)
            for pair,depth in pairs.items():
                if pair not in initial:bad[pair]=max(bad.get(pair,0.),depth)
        row=dict(kind=kind,prefix_goals=len(prefix),new_contacts_mm=bad,**escape.result())
        alternatives.append(row);print('ORDER',json.dumps(row),flush=True)
    (FOLDER/'opening_order_candidates.json').write_text(json.dumps(alternatives,indent=2),encoding='utf-8')


if __name__=='__main__':main()
