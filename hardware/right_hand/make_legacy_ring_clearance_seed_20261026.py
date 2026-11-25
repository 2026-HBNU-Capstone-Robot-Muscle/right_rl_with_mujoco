"""Check passive ring clearance before the proposed thumb/middle regrasp."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import copy,json
from pathlib import Path
import numpy as np,mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics
ROOT=Path(__file__).resolve().parent;SOURCE=ROOT/'right_hand_rotation_20261026/legacy_stage4_seating_v1'
def main():
    seed=json.loads((SOURCE/'seed.json').read_bytes());endpoints=json.loads((SOURCE/'thumb_opposed_pinch_endpoints.json').read_bytes());pairs=[p for p in endpoints['pairs'] if p['endpoint_promising']];pair=pairs[3];fail=json.loads((SOURCE/'adaptive_thumb_pair3_dir1_end_latched.json').read_bytes());end=np.load(SOURCE/'adaptive_thumb_pair3_dir1_end_latched.npz')['end_qpos'];residual=np.asarray(seed['response_residual_ticks']);state=np.asarray(seed['settled_cube_qpos'])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,_=build(env,SOURCE/'passive_ring_clearance_scene.xml');set_cube_physics(m,.055,.1);d=mujoco.MjData(m);cube=m.geom('cube').id;hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
    def check(goal,cube_state):
        d.qpos[:16]=ticks_to_sim(np.asarray(goal)+residual);d.qpos[-7:]=cube_state;d.qvel[:]=0;mujoco.mj_forward(m,d);cp=sp=sup=0.
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2);dep=max(0.,-float(c.dist)*1000);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
            if ha and hb:sp=max(sp,dep)
            if (a in support and hb) or (b in support and ha):sup=max(sup,dep)
            if cube in (a,b):cp=max(cp,dep)
        return dict(self_overlap_mm=sp,support_overlap_mm=sup,cube_overlap_mm=cp,clear=bool(sp<=.01 and sup<=.15 and cp<=1))
    rows=[]
    variants={'ring_uncurl':{12:2120,14:2304},'ring_outside':{13:1536},'ring_uncurl_outside':{12:2120,13:1536,14:2304},'ring_uncurl_other':{12:2120,13:2560,14:2304}}
    for name,changes in variants.items():
        def altered(q):
            a=np.asarray(q).copy()
            for i,v in changes.items():a[i]=v
            return a
        checks=dict(initial=check(altered(seed['commanded_hand_ticks']),state),grasp=check(altered(pair['goal_ticks']),state),blocked_turn=check(altered(fail['blocked_step']['tentative_goal']),end[-7:]));ok=all(v['clear'] for v in checks.values());row=dict(variant=name,changes_by_id={str(i+1):v for i,v in changes.items()},checks=checks,endpoint_promising=ok);rows.append(row)
        if ok:
            out=ROOT/'right_hand_rotation_20261026'/('legacy_stage4_'+name+'_v1');out.mkdir(exist_ok=True);s=copy.deepcopy(seed);s['commanded_hand_ticks']=altered(seed['commanded_hand_ticks']).tolist();s['actual_hand_ticks']=(np.asarray(s['commanded_hand_ticks'])+residual).tolist();s['passive_ring_pose_modified_from_photographed_pose']=True;s['passive_ring_changes_by_id']=row['changes_by_id'];s['passive_ring_release_path_and_response_not_verified']=True;(out/'seed.json').write_text(json.dumps(s,indent=2)+'\n',encoding='utf-8');ep=copy.deepcopy(endpoints)
            for entry in ep['thumb_candidates']:entry['goal_ticks']=altered(entry['goal_ticks']).tolist()
            for entry in ep['pairs']:
                entry['goal_ticks']=altered(entry['goal_ticks']).tolist();entry['thumb']['goal_ticks']=altered(entry['thumb']['goal_ticks']).tolist();entry['finger_candidate']['goal_ticks']=altered(entry['finger_candidate']['goal_ticks']).tolist()
            ep['passive_ring_variant']=row;ep['all_other_endpoints_need_rechecking']=True;(out/'thumb_opposed_pinch_endpoints.json').write_text(json.dumps(ep,indent=2)+'\n',encoding='utf-8')
    result=dict(status='passive_ring_clearance_endpoint_diagnostic',cases=rows,hardware_execution_allowed=False,real_devices_opened=False);(SOURCE/'passive_ring_clearance_review.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(result,indent=2),flush=True)
if __name__=='__main__':main()
