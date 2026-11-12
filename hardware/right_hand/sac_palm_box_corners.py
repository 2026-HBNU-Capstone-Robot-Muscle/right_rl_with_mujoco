"""Export simulated palm box corners at the measured stage-0 hand pose."""
import json
import sys
from pathlib import Path

import jax
import mujoco
import numpy as np
from mujoco_playground import registry

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'deploy'))
from leap_hw import hw_to_sim


def main():
    config = registry.get_default_config('LeapCubeFaceUp')
    config.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=config)
    model = env.mj_model
    q = np.asarray(env.reset(jax.random.PRNGKey(0)).data.qpos, dtype=float)
    trial = json.loads((ROOT / 'leap_support_capture_result.json').read_text(encoding='utf-8'))
    ticks = [row for row in trial['samples'] if row['phase']=='place_cube'][-1]['actual']
    q[:16] = hw_to_sim(np.asarray(ticks, dtype=float)*2*np.pi/4096)
    data = mujoco.MjData(model)
    data.qpos[:] = q
    mujoco.mj_forward(model, data)
    out = {}
    for suffix in (6,7,8):
        name = f'palm_collision_{suffix}'
        geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        center = data.geom_xpos[geom]
        rotation = data.geom_xmat[geom].reshape(3,3)
        size = model.geom_size[geom]
        corners = [center + rotation @ (size*np.asarray([sx, sy, sz]))
                   for sx in (-1,1) for sy in (-1,1) for sz in (-1,1)]
        out[name] = {'center_world_m': center.tolist(),
                     'half_extents_m': size.tolist(),
                     'corners_world_m': [point.tolist() for point in corners]}
    (ROOT / 'sac_palm_box_corners_result.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({name: {'center': item['center_world_m'],
                             'half_extents': item['half_extents_m']}
                      for name,item in out.items()},indent=2))


if __name__ == '__main__':
    main()
