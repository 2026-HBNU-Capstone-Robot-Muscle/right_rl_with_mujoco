"""Offline diagnostic of index/middle support near the measured stage-0 pose."""

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


def contact_geoms(model, data, cube_geom):
    result = []
    for contact in data.contact:
        if cube_geom in (contact.geom1, contact.geom2):
            other = contact.geom2 if contact.geom1 == cube_geom else contact.geom1
            result.append((float(contact.dist),
                           mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, other)))
    return result


def probe(model, q, cube_geom):
    data = mujoco.MjData(model)
    data.qpos[:] = q
    data.ctrl[:] = q[:16]
    mujoco.mj_forward(model, data)
    initial = contact_geoms(model, data, cube_geom)
    for _ in range(30):
        data.qpos[:16] = q[:16]
        data.qvel[:16] = 0
        mujoco.mj_forward(model, data)
        mujoco.mj_step(model, data)
    final = contact_geoms(model, data, cube_geom)
    return {
        'initial_geoms': sorted({name for _, name in initial}),
        'final_geoms': sorted({name for _, name in final}),
        'penetration_mm': float(-1000 * min([0.] + [d for d, _ in initial])),
        'drift_mm': float(1000 * np.linalg.norm(data.qpos[-7:-4] - q[-7:-4])),
    }


def both_fingers(names):
    return any(name.startswith('if_') for name in names) and any(
        name.startswith('mf_') for name in names)


def palm_support(names):
    return any(name.startswith('palm_') for name in names)


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
    candidates = {
        'valid_empty_capture_single_marker': 'sac_cross_session_191152_registration_result.json',
        'older_anchor': 'sac_registration_anchor_result.json',
    }
    rows = []
    for label, file in candidates.items():
        reg = json.loads((ROOT / file).read_text(encoding='utf-8'))
        center = np.asarray(reg['cube_center_in_sim_world_candidate_m'], dtype=float)
        q[-4:] = np.asarray(reg['cube_quat_world_candidate'], dtype=float)
        for dx in (-20, -10, 0, 10, 20):
            for dy in (-20, -10, 0, 10, 20):
                for dz in (-20, -10, 0, 10, 20):
                    q[-7:-4] = center + np.array([dx, dy, dz]) / 1000
                    result = probe(model, q, cube_geom)
                    rows.append({'orientation': label, 'offset_mm': [dx, dy, dz],
                                 'center_m': q[-7:-4].tolist(), **result})
    initial_both = [row for row in rows if both_fingers(row['initial_geoms'])]
    sustained_both = [row for row in initial_both if both_fingers(row['final_geoms'])]
    plausible = [row for row in sustained_both if row['penetration_mm'] <= 2
                 and row['drift_mm'] <= 10]
    initial_palm = [row for row in rows if palm_support(row['initial_geoms'])]
    sustained_palm = [row for row in initial_palm if palm_support(row['final_geoms'])]
    plausible_palm = [row for row in sustained_palm if row['penetration_mm'] <= 2
                      and row['drift_mm'] <= 10]
    report = {
        'status': 'offline_contact_diagnostic',
        'hardware_authorized': False,
        'cube_mass_kg': .100,
        'trial_file': 'leap_sequence_flip_20261008_191152_267924.json',
        'samples': len(rows),
        'initial_index_middle_count': len(initial_both),
        'sustained_index_middle_count': len(sustained_both),
        'plausible_count': len(plausible),
        'initial_palm_support_count': len(initial_palm),
        'sustained_palm_support_count': len(sustained_palm),
        'plausible_palm_support_count': len(plausible_palm),
        'best_palm_support': sorted(sustained_palm,
                                    key=lambda r: (r['penetration_mm'] > 2,
                                                   r['drift_mm']))[:20],
        'best_plausible_palm_support': sorted(plausible_palm,
                                             key=lambda r: r['drift_mm'])[:20],
        'best_sustained': sorted(sustained_both,
                                 key=lambda r: (r['penetration_mm'] > 2,
                                                r['drift_mm']))[:12],
        'best_plausible': sorted(plausible, key=lambda r: r['drift_mm'])[:12],
        'caveat': 'Two one-marker orientation candidates and a coarse 10 mm grid. Palm contact is now the user-reported primary support; finger contact is optional. This does not establish real cube pose or deployment readiness.',
    }
    out = ROOT / 'sac_index_middle_support_probe_result.json'
    out.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({k: v for k, v in report.items() if k not in
                      ('best_sustained', 'best_plausible', 'best_palm_support',
                       'best_plausible_palm_support')}, indent=2))
    print('best_sustained', report['best_sustained'][:3])
    print('best_palm_support', report['best_palm_support'][:3])


if __name__ == '__main__':
    main()
