"""Current OFF geometry diagnostic. No devices or execution approval."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import numpy as np,mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from loaded_quarter_reentry_20261025 import limits
ROOT=Path(__file__).resolve().parent
def main():
    latest_path=ROOT/'right_hand_rotation_20261026/fresh_empty_start_latest.json';latest=json.loads(latest_path.read_bytes())
    for key in ('folder','result'):latest[key]=latest[key].replace('\\','/')
    latest_path.write_text(json.dumps(latest,indent=2)+'\n',encoding='utf-8');out=ROOT/latest['folder'];raw=np.asarray(latest['position_ticks']);cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,manifest=build(env,out/'current_off_scene.xml');d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(raw);mujoco.mj_forward(m,d);hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    external={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};selfcontacts=[];obstacles=[]
    for c in d.contact:
        a,b=int(c.geom1),int(c.geom2);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand;depth=-float(c.dist)*1000
        if depth<=1e-6:continue
        row=dict(a=m.geom(a).name,b=m.geom(b).name,depth_mm=depth)
        if ha and hb:selfcontacts.append(row)
        if (a in external and hb) or (b in external and ha):obstacles.append(row)
    lo,hi=limits();r=dict(status='fresh_off_model_geometry',source=latest['result'],position_ticks=raw.tolist(),native_goal_range_valid=bool(np.all(raw>=lo) and np.all(raw<=hi)),range_violations=[dict(motor_id=i+1,value=int(raw[i]),lower=float(lo[i]),upper=float(hi[i])) for i in range(16) if raw[i]<lo[i] or raw[i]>hi[i]],self_contacts=selfcontacts,obstacle_contacts=obstacles,model_manifest=manifest,physical_contacts_not_independently_certified=True,hardware_motion_allowed=False,real_devices_opened=False);(out/'current_off_model_review.json').write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in r.items() if k!='model_manifest'},indent=2),flush=True)
if __name__=='__main__':main()
