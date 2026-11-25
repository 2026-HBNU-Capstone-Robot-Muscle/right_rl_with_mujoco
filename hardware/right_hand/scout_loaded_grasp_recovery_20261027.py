"""Offline recovery scout from the settled, measured post-grasp OFF pose."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import mujoco,numpy as np
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from loaded_quarter_model_policy_20261025 import correct_model
from loaded_quarter_reentry_20261025 import limits,validate_path
from hand_motor_mapping import ticks_to_sim
from fresh_empty_grasp_plan_20261027 import interpolate
from screen_loaded_repeat_20261026 import cached_contacts
ROOT=Path(__file__).resolve().parent
def main():
    latest=json.loads((ROOT/'right_hand_rotation_20261027/grasp_off_latest.json').read_bytes());raw=np.array(json.loads((ROOT/latest['result']).read_bytes())['position_ticks']);out=ROOT/'right_hand_rotation_20261027/loaded_grasp_recovery';out.mkdir(exist_ok=True);cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);cases=[]
    for front in (-15.,-10.,-5.):
        m,manifest=build(env,out/f'scene_{front:g}.xml',book_projection_mm=front);range_policy=correct_model(m,raw);m.geom_contype[m.geom_bodyid==m.body('cube').id]=0;m.geom_conaffinity[m.geom_bodyid==m.body('cube').id]=0;d=mujoco.MjData(m);hand={m.body('palm').id}
        for b in range(m.nbody):
            if int(m.body_parentid[b]) in hand:hand.add(b)
        support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};inspect=cached_contacts(m,hand,support)
        def obs(q):d.qpos[:16]=ticks_to_sim(q);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];d.qvel[:]=0;mujoco.mj_forward(m,d);return inspect(d)
        initial_su,initial_sp=obs(raw);options=[]
        for tip in (0,-32,-64):
            entry=raw.copy();entry[3]=1872;entry[0]+=tip;entry_rows=interpolate(raw,entry);previous_sp=initial_sp;previous_su=initial_su;spbad=subad=0.;first_self_clear=0 if not initial_sp else None
            for i,q in enumerate(entry_rows):
                su,sp=obs(q);spbad=max(spbad,max((v-previous_sp.get(k,0.) for k,v in sp.items()),default=0.));subad=max(subad,max((v-previous_su.get(k,0.) for k,v in su.items()),default=0.));previous_sp=sp;previous_su=su
                if first_self_clear is None and not sp:first_self_clear=i+1
            for height in (2150,2250,2472):
                goal=entry.copy();goal[[7,11]]=height;goal[15]=2053;previous_su=initial_su;bad=spbad;supportbad=subad;clear=None;rows=interpolate(entry,goal)
                for i,q in enumerate(rows):
                    su,sp=obs(q);bad=max(bad,max(sp.values(),default=0.));supportbad=max(supportbad,max((v-previous_su.get(k,0.) for k,v in su.items()),default=0.));previous_su=su
                    if clear is None and max(su.values(),default=0.)<=.15:clear=i+1
                queue=[('entry',q) for q in entry_rows]+[('roots',q) for q in rows];validate_path(raw,queue,'entry');options.append(dict(tip_delta=tip,root_target=height,entry_samples=len(entry_rows),root_samples=len(rows),initial_self_clear_sample=first_self_clear,max_self_increase_mm=max(0,spbad),max_self_after_entry_mm=max(0,bad),support_increase_mm=max(0,supportbad),support_clear_sample=clear,end_ticks=goal.tolist()))
        cases.append(dict(front=front,initial_support=initial_su,initial_self=initial_sp,existing_passive_reentry=range_policy,options=options));print('FRONT',front,'initial',initial_su,initial_sp,flush=True)
    (out/'scout_rf_clear.json').write_text(json.dumps(dict(raw_ticks=raw.tolist(),readonly_source=latest,cases=cases,devices_opened=False,hardware_motion_allowed=False),indent=2)+'\n',encoding='utf-8');print(json.dumps([dict(front=c['front'],options=[{k:v for k,v in o.items() if k!='end_ticks'} for o in c['options']]) for c in cases],indent=2),flush=True)
if __name__=='__main__':main()
