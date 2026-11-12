"""Search nearby starts in a recompiled visual-palm model; offline only."""
import json
from pathlib import Path
import sys

import jax
import numpy as np
from mujoco_playground import registry

from sac_palm_recompiled_collision_probe import make_model, run

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work/Leap_Test/deploy'))
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
    reg = json.loads((ROOT / 'sac_cross_session_latest_registration_result.json').read_text(encoding='utf-8'))
    base = np.asarray(reg['cube_center_in_sim_world_candidate_m'])
    q[-4:] = reg['cube_quat_world_candidate']
    rows = []
    for dx in (-20, -10, 0, 10, 20):
        for dy in (-15, -10, -5, 0, 5, 10, 15):
            for dz in (-15, -10, -5, 0, 5, 10, 15):
                q[-7:-4] = base + np.array([dx, dy, dz]) / 1000
                rows.append({'offset_mm': [dx, dy, dz], **run(model, q)})
    admissible = [r for r in rows if r['initial_contacts'] and r['final_contacts']
                  and r['initial_penetration_mm'] <= 2]
    admissible.sort(key=lambda r: (r['drift_mm'], np.linalg.norm(r['offset_mm'])))
    output = {'hardware_authorized': False, 'model': 'visual_without_boxes',
              'base_center_m': base.tolist(), 'samples': len(rows),
              'admissible_count': len(admissible), 'best_admissible': admissible[:15],
              'best_under_10mm_drift_count': sum(r['drift_mm'] < 10 for r in admissible),
              'caveat': 'A fitted collision point is not camera calibration or physical flip evidence. Some palm boxes are excluded only in this offline model variant.'}
    path = ROOT / 'sac_oct8_local_contact_search_result.json'
    path.write_text(json.dumps(output, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({k: v for k, v in output.items() if k != 'best_admissible'}, indent=2))
    print('best_admissible', admissible[:5])


if __name__ == '__main__':
    main()
