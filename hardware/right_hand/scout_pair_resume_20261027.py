"""No devices: screen the latest stopped OFF pose directly toward OPEN."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
import numpy as np,mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from loaded_quarter_model_policy_20261025 import correct_model
from new_pair_grasp_plan_20261027 import ROOT,OPEN_PATH,ENTRY,ROOTS,OPEN
from fresh_empty_grasp_plan_20261027 import interpolate
from hand_motor_mapping import ticks_to_sim
from screen_loaded_repeat_20261026 import cached_contacts
from book_empty_preparation_plan import FRONTS
def main():
    folder=ROOT/'right_hand_rotation_20261027/pair_resume_after_place_timeout';folder.mkdir(exist_ok=True);source=ROOT/'new_pair_grasp_20261027_045408_418203/fresh_off_request.json';raw=np.array(json.loads(source.read_bytes())['off_state']['position']);entry=raw.copy();entry[3]=max(entry[3],1872);root=entry.copy();root[[7,11]]=np.maximum(root[[7,11]],2150);root[15]=max(root[15],2053);target=np.load(OPEN_PATH)['open_goal'];rows=[(ENTRY,q) for q in interpolate(raw,entry)]+[(ROOTS,q) for q in interpolate(entry,root,1)]+[(OPEN,q) for q in interpolate(root,target)];cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);report=[]
    for front in FRONTS:
        m,_=build(env,folder/f'direct_{front:g}.xml',book_projection_mm=front);correct_model(m,raw);cube=m.geom_bodyid==m.body('cube').id;m.geom_contype[cube]=0;m.geom_conaffinity[cube]=0;hand={m.body('palm').id}
        for b in range(m.nbody):
            if int(m.body_parentid[b]) in hand:hand.add(b)
        support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};inspect=cached_contacts(m,hand,support);d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(raw);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];mujoco.mj_forward(m,d);initial=inspect(d);worst={};bad=[]
        for index,(phase,q) in enumerate(rows):
            d.qpos[:16]=ticks_to_sim(q);mujoco.mj_forward(m,d);su,sp=inspect(d)
            for pair,value in sp.items():worst[pair]=max(worst.get(pair,0),value)
            if max(sp.values(),default=0)>1e-6 or (phase==OPEN and max(su.values(),default=0)>.15):
                if len(bad)<5:bad.append(dict(index=index,phase=phase,self_contacts=sp,support=su))
        report.append(dict(front=front,initial_support=initial[0],initial_self=initial[1],worst_self=worst,first_bad=bad,goals=len(rows)));print(json.dumps(report[-1]),flush=True)
    (folder/'direct_scout.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
