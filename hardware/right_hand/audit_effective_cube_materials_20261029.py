"""Inspect actual generated contact coefficients. No devices or motion API."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from calibrated_quarter_plan_20261028 import OPEN_PATH
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics
from cube_contact_material_policy_20261029 import apply_material,cube_contact_friction

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261029/effective_material_audit'
def main():
    OUT.mkdir(exist_ok=True)
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg)
    model,_=build(env,OUT/'scene.xml',book_projection_mm=-10.)
    set_cube_physics(model,.055,.1)
    hand={model.body('palm').id}
    for body in range(model.nbody):
        if int(model.body_parentid[body]) in hand:hand.add(body)
    cube=model.geom('cube').id
    original_friction=model.geom_friction.copy()
    original_pairs=model.pair_friction.copy()
    prior=json.loads((ROOT/'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json').read_bytes())
    seed=next(r for r in prior['loaded_physics'] if (r['book_front_mm'],r['gain_scale'],r['friction'])==(-10.,1.,.3))
    data=mujoco.MjData(model)
    data.qpos[:16]=ticks_to_sim(np.load(OPEN_PATH)['open_goal'])
    data.qpos[-7:]=seed['settled_cube_qpos']
    reports=[]
    for corrected in (False,True):
      for value in (.2,.3,.5):
        model.geom_friction[:]=original_friction;model.pair_friction[:]=original_pairs
        if corrected:manifest=apply_material(model,value,hand,cube)
        else:model.geom_friction[cube,0]=value;manifest=None
        mujoco.mj_forward(model,data)
        contacts=cube_contact_friction(model,data,hand,cube)
        row=dict(both_surfaces_policy=corrected,requested=value,contacts=contacts,manifest=manifest)
        if corrected and (not contacts or any(not np.allclose(c['sliding'],value,rtol=0,atol=1e-12) for c in contacts)):
            raise RuntimeError('Effective contact coefficient differs from requested material')
        reports.append(row)
    result=dict(cases=reports,hardware_execution_allowed=False,devices_opened=False,
        official_reference='https://mujoco.readthedocs.io/en/stable/modeling.html#contact-parameters')
    (OUT/'review.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    for row in reports:print(json.dumps({k:v for k,v in row.items() if k!='manifest'}),flush=True)

if __name__=='__main__':main()
