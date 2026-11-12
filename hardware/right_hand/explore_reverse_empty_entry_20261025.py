"""Compare full-CAD EMPTY entry stages from the fresh actual OFF state."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from observed_book_model_policy import correct_model
from hand_motor_mapping import ticks_to_sim
from grasp_place_plan import endpoints
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/reverse_empty_entry'
SOURCE='reverse_start_readonly_20261025_040743_585738/result.json'

def interpolate(a,b):
    n=max(1,int(np.ceil(np.max(abs(b-a))/2)))
    return np.rint(a+(b-a)*np.arange(1,n+1)[:,None]/n).astype(int)

def main():
    OUT.mkdir(exist_ok=True);s=json.loads((ROOT/SOURCE).read_bytes());a=np.array(s['off_state']['position'])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build(env,OUT/'entry_CAD.xml');manifest['observed_model_correction']=correct_model(m,a)
    d=mujoco.MjData(m);cube=m.geom('cube').id;m.geom_contype[cube]=m.geom_conaffinity[cube]=0
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    supports={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
    def contacts(raw):
        d.qpos[:16]=ticks_to_sim(raw);d.qpos[-7:]=[10,10,10,1,0,0,0];d.qvel[:]=0;mujoco.mj_forward(m,d);result={}
        for c in d.contact:
            if c.dist>=0:continue
            x,y=int(c.geom1),int(c.geom2);hx,hy=int(m.geom_bodyid[x]) in hand,int(m.geom_bodyid[y]) in hand
            if hx and hy or x in supports and hy or y in supports and hx:
                pair='|'.join(sorted((m.geom(x).name,m.geom(y).name)));result[pair]=max(result.get(pair,0.),-float(c.dist)*1000)
        return result
    ring=a.copy();ring[14]-=80;thumb=ring.copy();thumb[0]-=64
    roots=thumb.copy();roots[[7,11,15]]=np.maximum(roots[[7,11,15]],2472)
    opened,_=endpoints()
    plan=json.loads((ROOT/'right_hand_rotation_20261024/original_sequence_clearance/CAD_adapted_forward_reference_path_v2.json').read_bytes())
    final=np.array(plan['dense_goal_ticks'][-1]);results=[];rows=[]
    print('CURRENT_CONTACTS',contacts(a),flush=True)
    previous=a
    for name,target in [('ring_release',ring),('thumb_release',thumb),('root_lift',roots),('opened',opened),('source_pose_2',final)]:
        maxima={};initial=contacts(previous);samples=interpolate(previous,target)
        for raw in samples:
            for pair,depth in contacts(raw).items():maxima[pair]=max(maxima.get(pair,0.),depth)
        result=dict(phase=name,goals=len(samples),initial_contacts_mm=initial,maximum_contacts_mm=maxima,final_contacts_mm=contacts(target))
        results.append(result);rows.extend((name,q.tolist()) for q in samples);previous=target
        print('EMPTY_ENTRY_STAGE',json.dumps(result),flush=True)
    (OUT/'direct_stage_audit.json').write_text(json.dumps(dict(source=SOURCE,source_raw_ticks=a.tolist(),
        target_raw_ticks=final.tolist(),stages=results,rows=rows,model=manifest,hardware_execution_allowed=False,
        hardware_commands_sent=0),indent=2),encoding='utf-8')

if __name__=='__main__':main()
