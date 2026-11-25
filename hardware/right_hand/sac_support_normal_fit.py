"""Offline sweep of small support-normal placement error and contact friction."""
import json
import sys
from pathlib import Path

import jax
import mujoco
import numpy as np
from mujoco_playground import registry

from sac_cube_model import set_cube_physics

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parent/'work'/'Leap_Test'/'deploy'))
from leap_hw import hw_to_sim


def main():
    config=registry.get_default_config('LeapCubeFaceUp')
    config.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=config)
    model=env.mj_model
    set_cube_physics(model,mass_kg=.100)
    q=np.asarray(env.reset(jax.random.PRNGKey(0)).data.qpos,dtype=float)
    trial=json.loads((ROOT/'leap_support_capture_result.json').read_text(encoding='utf-8'))
    ticks=[row for row in trial['samples'] if row['phase']=='place_cube'][-1]['actual']
    q[:16]=hw_to_sim(np.asarray(ticks)*2*np.pi/4096)
    placement=json.loads((ROOT/'sac_registration_support_photo_diagnostic.json').read_text(encoding='utf-8'))
    base=np.asarray(placement['cube_center_in_sim_world_candidate_m'],dtype=float)
    q[-4:]=placement['cube_quat_world_candidate']
    cube=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'cube')
    original_friction=model.geom_friction[cube].copy()
    support_normal=np.array([.3395997574287447,0.,.9405700424499699])
    rows=[]
    for lift_mm in (0,2,3,4,5,6,8,10):
        q[-7:-4]=base+support_normal*(lift_mm/1000)
        for friction in (.3,.6,1.0,2.0):
            for lock_hand in (False,True):
                model.geom_friction[cube,0]=friction
                data=mujoco.MjData(model)
                data.qpos[:]=q
                data.ctrl[:]=q[:16]
                mujoco.mj_forward(model,data)
                def contacts():
                    found=[]
                    for item in data.contact:
                        if cube in (item.geom1,item.geom2):
                            other=item.geom2 if item.geom1==cube else item.geom1
                            found.append((float(item.dist),mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,other)))
                    return found
                initial=contacts()
                for _ in range(30):
                    if lock_hand:
                        data.qpos[:16]=q[:16]
                        data.qvel[:16]=0
                        mujoco.mj_forward(model,data)
                    mujoco.mj_step(model,data)
                final=contacts()
                rows.append({'lift_normal_mm':lift_mm,'cube_friction':friction,
                             'lock_hand':lock_hand,
                             'initial_penetration_mm':-1000*min([0.]+[v[0] for v in initial]),
                             'initial_contacts':len(initial),'final_contacts':len(final),
                             'final_geoms':sorted({v[1] for v in final}),
                             'drift_mm':float(1000*np.linalg.norm(data.qpos[-7:-4]-q[-7:-4]))})
    model.geom_friction[cube]=original_friction
    result={'support_normal_world':support_normal.tolist(), 'rows':rows,
            'hardware_authorized':False,
            'caveat':'Unmeasured lift and friction are sensitivity only, not calibrated settings.'}
    (ROOT/'sac_support_normal_fit_result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'best_with_initial_contact':sorted((r for r in rows if r['initial_contacts']>0),
                                                      key=lambda r:r['drift_mm'])[:8],
                      'rows':len(rows)},indent=2))


if __name__=='__main__':
    main()
