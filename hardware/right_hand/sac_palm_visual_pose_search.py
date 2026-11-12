"""Offline local placement sweep with a separately recompiled palm mesh model."""
import json
import sys
from pathlib import Path

import jax
import numpy as np
from mujoco_playground import registry

from sac_palm_recompiled_collision_probe import make_model, run


ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'deploy'))
from leap_hw import hw_to_sim


def main():
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=cfg)
    model, _ = make_model(env, 'visual_without_boxes')
    q = np.asarray(env.reset(jax.random.PRNGKey(0)).data.qpos, dtype=float)
    trial = json.loads((ROOT / 'leap_support_capture_result.json').read_text(encoding='utf-8'))
    ticks = [r for r in trial['samples'] if r['phase'] == 'place_cube'][-1]['actual']
    q[:16] = hw_to_sim(np.asarray(ticks) * 2 * np.pi / 4096)
    anchor = json.loads((ROOT / 'sac_registration_support_photo_diagnostic.json').read_text(encoding='utf-8'))
    base = np.asarray(anchor['cube_center_in_sim_world_candidate_m'], dtype=float)
    q[-4:] = anchor['cube_quat_world_candidate']
    rows = []
    for dx_mm in (-20, -15, -10, -5, 0, 5, 10, 15, 20):
        for dz_mm in (-5, 0, 5, 10, 15, 20):
            q[-7:-4] = base + np.array([dx_mm, 0, dz_mm]) / 1000
            rows.append({'dx_mm': dx_mm, 'dz_mm': dz_mm, **run(model, q)})
    plausible = [r for r in rows if r['initial_contacts'] and r['final_contacts']
                 and r['initial_penetration_mm'] <= 2]
    refined = []
    for dx_mm in range(5, 16):
        for dz_mm in range(-4, 5):
            q[-7:-4] = base + np.array([dx_mm, 0, dz_mm]) / 1000
            refined.append({'dx_mm': dx_mm, 'dz_mm': dz_mm, **run(model, q)})
    refined_plausible = [r for r in refined if r['initial_contacts'] and r['final_contacts']
                         and r['initial_penetration_mm'] <= 2]
    output = {'rows': rows, 'plausible_contact_count': len(plausible),
              'best_plausible_contact': sorted(plausible, key=lambda r: r['drift_mm'])[:10],
              'refined_rows': refined,
              'refined_plausible_contact_count': len(refined_plausible),
              'best_refined_plausible_contact': sorted(refined_plausible, key=lambda r: r['drift_mm'])[:10],
              'hardware_authorized': False,
              'caveat': 'This mesh model and the single-marker placement are unvalidated; no deployment inference.'}
    (ROOT / 'sac_palm_visual_pose_search_result.json').write_text(json.dumps(output, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'plausible_contact_count': len(plausible),
                      'best_plausible_contact': output['best_plausible_contact'][:3],
                      'refined_plausible_contact_count': len(refined_plausible),
                      'best_refined_plausible_contact': output['best_refined_plausible_contact'][:5]}, indent=2))


if __name__ == '__main__':
    main()
