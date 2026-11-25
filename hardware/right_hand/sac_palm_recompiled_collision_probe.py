"""Recompile separate offline MuJoCo models to test palm mesh collision."""
import argparse
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import jax
import mujoco
import numpy as np
from mujoco_playground import registry

from sac_cube_model import set_cube_physics


ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'deploy'))
from leap_hw import hw_to_sim


def make_model(env, setting):
    assets = env._model_assets.copy()
    source = assets['leap_rh_mjx.xml']
    root = ET.fromstring(source)
    geoms = {g.get('name'): g for g in root.iter('geom') if g.get('name')}
    if setting.startswith('visual'):
        geoms['palm_visual'].set('contype', '1')
        geoms['palm_visual'].set('conaffinity', '1')
    if setting in ('visual_without_boxes', 'without_boxes'):
        for i in (6, 7, 8):
            g = geoms['palm_collision_' + str(i)]
            g.set('contype', '0')
            g.set('conaffinity', '0')
    assets['leap_rh_mjx.xml'] = ET.tostring(root)
    scene = Path(env.xml_path).read_text()
    model = mujoco.MjModel.from_xml_string(scene, assets=assets)
    set_cube_physics(model, mass_kg=.100)
    return model, hashlib.sha256(source).hexdigest()


def run(model, q):
    data = mujoco.MjData(model)
    data.qpos[:] = q
    data.ctrl[:] = q[:16]
    cube = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    mujoco.mj_forward(model, data)
    def contacts():
        found = []
        for c in data.contact:
            if cube in (c.geom1, c.geom2):
                other = c.geom2 if c.geom1 == cube else c.geom1
                found.append((float(c.dist), mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, other)))
        return found
    initial = contacts()
    for _ in range(30):
        data.qpos[:16] = q[:16]
        data.qvel[:16] = 0
        mujoco.mj_forward(model, data)
        mujoco.mj_step(model, data)
    final = contacts()
    return {'initial_contacts': len(initial),
            'initial_geoms': sorted({v[1] for v in initial}),
            'initial_penetration_mm': -1000 * min([0.] + [v[0] for v in initial]),
            'final_contacts': len(final), 'final_geoms': sorted({v[1] for v in final}),
            'drift_mm': float(1000 * np.linalg.norm(data.qpos[-7:-4] - q[-7:-4]))}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--registration-file', default='sac_registration_support_photo_diagnostic.json')
    parser.add_argument('--output-file', default='sac_palm_recompiled_collision_probe_result.json')
    parser.add_argument('--trial-file', default='leap_support_capture_result.json')
    args = parser.parse_args()
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=cfg)
    state = env.reset(jax.random.PRNGKey(0))
    q = np.asarray(state.data.qpos, dtype=float)
    trial = json.loads((ROOT / args.trial_file).read_text(encoding='utf-8'))
    ticks = [r for r in trial['samples'] if r['phase'] == 'place_cube'][-1]['actual']
    q[:16] = hw_to_sim(np.asarray(ticks) * 2 * np.pi / 4096)
    anchor = json.loads((ROOT / args.registration_file).read_text(encoding='utf-8'))
    q[-7:-4] = anchor['cube_center_in_sim_world_candidate_m']
    q[-4:] = anchor['cube_quat_world_candidate']
    rows = []
    source_hash = None
    for setting in ('original', 'visual_plus_boxes', 'visual_without_boxes', 'without_boxes'):
        model, source_hash = make_model(env, setting)
        rows.append({'setting': setting, **run(model, q)})
    output = {'hand_xml_sha256': source_hash, 'rows': rows, 'hardware_authorized': False,
              'caveat': 'The visual mesh is interpreted by MuJoCo collision rules, not validated against the physical hand.'}
    (ROOT / args.output_file).write_text(json.dumps(output, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(output, indent=2))


if __name__ == '__main__':
    main()
