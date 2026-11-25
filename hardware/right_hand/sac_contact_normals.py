"""Report measured-start MuJoCo contact directions and overlap, offline."""
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
    q[-7:-4]=placement['cube_center_in_sim_world_candidate_m']
    q[-4:]=placement['cube_quat_world_candidate']
    data=mujoco.MjData(model)
    data.qpos[:]=q
    data.ctrl[:]=q[:16]
    mujoco.mj_forward(model,data)
    cube=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'cube')
    rows=[]
    for item in data.contact:
        if cube not in (item.geom1,item.geom2):
            continue
        other=item.geom2 if item.geom1==cube else item.geom1
        rows.append({'other_geom':mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,other),
                     'penetration_mm':max(0.,-1000*float(item.dist)),
                     'point_world_m':item.pos.tolist(),
                     'normal_geom1_to_geom2_world':item.frame[:3].tolist(),
                     'cube_is_geom1':bool(item.geom1==cube)})
    result={'contacts':rows,'hardware_authorized':False}
    (ROOT/'sac_contact_normals_result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
