"""Trace offline cube motion from the photographed support pose."""
import json
import sys
from pathlib import Path

import jax
import mujoco
import numpy as np
from mujoco_playground import registry

from sac_cube_model import set_cube_physics


ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'deploy'))
from leap_hw import hw_to_sim


def main():
    config = registry.get_default_config('LeapCubeFaceUp')
    config.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=config)
    model = env.mj_model
    set_cube_physics(model, mass_kg=.100)
    q = np.asarray(env.reset(jax.random.PRNGKey(0)).data.qpos, dtype=float)
    trial = json.loads((ROOT / 'leap_support_capture_result.json').read_text(encoding='utf-8'))
    ticks = [r for r in trial['samples'] if r['phase'] == 'place_cube'][-1]['actual']
    q[:16] = hw_to_sim(np.asarray(ticks) * 2 * np.pi / 4096)
    anchor = json.loads((ROOT / 'sac_registration_support_photo_diagnostic.json').read_text(encoding='utf-8'))
    q[-7:-4] = anchor['cube_center_in_sim_world_candidate_m']
    q[-4:] = anchor['cube_quat_world_candidate']
    cube = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    support_normal = np.array([.3395997574287447, 0., .9405700424499699])
    rows = []
    for lift_mm, friction in [(0, .3), (3, .3), (3, 2.), (6, 2.)]:
        model.geom_friction[cube, 0] = friction
        data = mujoco.MjData(model)
        data.qpos[:] = q
        data.qpos[-7:-4] += support_normal * lift_mm / 1000
        initial = data.qpos[-7:-4].copy()
        data.ctrl[:] = q[:16]
        mujoco.mj_forward(model, data)
        trace = []
        for step in range(31):
            if step % 5 == 0:
                geoms = []
                for c in data.contact:
                    if cube in (c.geom1, c.geom2):
                        other = c.geom2 if c.geom1 == cube else c.geom1
                        geoms.append(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, other))
                trace.append({'t_s': round(step * model.opt.timestep, 3),
                              'position_mm': (1000 * data.qpos[-7:-4]).round(2).tolist(),
                              'delta_mm': (1000 * (data.qpos[-7:-4] - initial)).round(2).tolist(),
                              'velocity_m_s': data.qvel[-6:-3].round(3).tolist(),
                              'contacts': len(geoms), 'geoms': sorted(set(geoms))})
            if step < 30:
                data.qpos[:16] = q[:16]
                data.qvel[:16] = 0
                mujoco.mj_forward(model, data)
                mujoco.mj_step(model, data)
        rows.append({'lift_mm': lift_mm, 'friction': friction, 'trace': trace})
    output = {'model_gravity_m_s2': model.opt.gravity.tolist(), 'model_timestep_s': model.opt.timestep,
              'rows': rows, 'hardware_authorized': False}
    (ROOT / 'sac_support_motion_trace_result.json').write_text(json.dumps(output, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(output, indent=2))


if __name__ == '__main__':
    main()
