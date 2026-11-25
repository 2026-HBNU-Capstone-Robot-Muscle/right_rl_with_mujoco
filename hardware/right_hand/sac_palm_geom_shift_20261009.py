"""Offline sensitivity to palm collision-box placement at measured stage 0."""

import json
import sys
from pathlib import Path

import jax
import mujoco
import numpy as np
from mujoco_playground import registry

from hand_registration_geometry import candidate
from sac_cube_model import set_cube_physics
from sac_index_middle_support_probe import probe


ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'deploy'))
from leap_hw import hw_to_sim


def main():
    reg = json.loads((ROOT / 'sac_registration_anchor_result.json').read_text(encoding='utf-8'))
    trial = json.loads((ROOT / 'leap_sequence_flip_20261009_221628_602843.json').read_text(encoding='utf-8'))
    ticks = [r for r in trial['samples'] if r['phase'] == 'place_cube'][-1]['actual']
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=cfg)
    model = env.mj_model
    set_cube_physics(model, mass_kg=.100)
    cube_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    q = np.asarray(env.reset(jax.random.PRNGKey(0)).data.qpos, dtype=float)
    q[:16] = hw_to_sim(np.asarray(ticks) * 2 * np.pi / 4096)
    q[-7:-4] = reg['cube_center_in_sim_world_candidate_m']
    q[-4:] = reg['cube_quat_world_candidate']
    normal = np.asarray(candidate()['hand_to_world_candidate']) @ [0., 0., 1.]
    data = mujoco.MjData(model)
    data.qpos[:] = q
    mujoco.mj_forward(model, data)
    geoms = [g for g in range(model.ngeom)
             if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or '').startswith('palm_collision_')]
    original = model.geom_pos.copy()
    rows = []
    for offset_mm in (0, 2, 4, 6, 8, 10, 12):
        model.geom_pos[:] = original
        for g in geoms:
            body = model.geom_bodyid[g]
            direction_local = data.xmat[body].reshape(3, 3).T @ normal
            model.geom_pos[g] -= direction_local * offset_mm / 1000
        rows.append({'palm_boxes_lowered_mm': offset_mm, **probe(model, q, cube_geom)})
    model.geom_pos[:] = original
    report = {'status': 'offline_palm_box_sensitivity', 'hardware_authorized': False,
              'palm_geom_count': len(geoms), 'normal_world': normal.tolist(), 'rows': rows,
              'caveat': 'Hypothetical in-memory collision geometry; no evidence yet that physical palm boxes are this low.'}
    (ROOT / 'sac_palm_geom_shift_20261009_result.json').write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
