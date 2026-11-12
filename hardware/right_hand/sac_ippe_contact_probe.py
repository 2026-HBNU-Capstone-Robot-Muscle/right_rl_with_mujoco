"""Offline palm-contact test of both one-marker IPPE cube poses."""

import json
import sys
from pathlib import Path

import jax
import mujoco
import numpy as np
from mujoco_playground import registry

from sac_cube_model import set_cube_physics
from sac_index_middle_support_probe import probe


ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'deploy'))
from leap_hw import hw_to_sim


def main():
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=cfg)
    model = env.mj_model
    set_cube_physics(model, mass_kg=.100)
    cube_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    trial = json.loads((ROOT / 'leap_sequence_flip_20261008_191152_267924.json').read_text(encoding='utf-8'))
    ticks = [row for row in trial['samples'] if row['phase'] == 'place_cube'][-1]['actual']
    q = np.asarray(env.reset(jax.random.PRNGKey(0)).data.qpos, dtype=float)
    q[:16] = hw_to_sim(np.asarray(ticks) * 2 * np.pi / 4096)
    source = json.loads((ROOT / 'sac_single_marker_ippe_candidates_result.json').read_text(encoding='utf-8'))
    rows = []
    for entry in source['solutions']:
        q[-7:-4] = entry['cube_center_in_sim_world_candidate_m']
        q[-4:] = entry['cube_quat_world_candidate']
        rows.append({'solution': entry['solution'],
                     'green_face5_up_dot': entry['visible_green_face5_up_dot_candidate'],
                     'reprojection_px': entry['reprojection_px'],
                     **probe(model, q, cube_geom)})
    report = {'status': 'offline_ippe_contact_diagnostic', 'hardware_authorized': False,
              'trial_file': 'leap_sequence_flip_20261008_191152_267924.json',
              'rows': rows,
              'caveat': 'The marker-to-model transform remains approximate; PnP reprojection alone cannot validate contact.'}
    (ROOT / 'sac_ippe_contact_probe_result.json').write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
