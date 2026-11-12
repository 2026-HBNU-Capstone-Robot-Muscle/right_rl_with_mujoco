"""Offline local search for two-finger cube contact at the recorded first pose."""

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
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=cfg)
    model = env.mj_model
    set_cube_physics(model, mass_kg=.100)
    tilt = json.loads((ROOT / 'table_tilt_20261010_053815_726640' /
                       'tilt_result.json').read_text())
    model.opt.gravity[:] = tilt['gravity_in_sim_world_m_s2_candidate']
    reg = json.loads((ROOT / 'sac_cube_rim_register_1010_stage0_result.json').read_text())
    saved = json.loads((ROOT / 'leap_support_capture_result.json').read_text())
    actual = next(s for s in reversed(saved['samples']) if s['phase'] == 'place_cube')['actual']
    start_q = np.asarray(env.reset(jax.random.PRNGKey(0)).data.qpos).copy()
    start_q[:16] = hw_to_sim(np.asarray(actual) * 2 * np.pi / 4096)
    start_q[-4:] = reg['cube_quat_world_candidate']
    center = np.asarray(reg['cube_center_world_m'])
    cube_body = model.body('cube').id
    data = mujoco.MjData(model)
    results = []
    values = np.arange(-10, 10.01, 2.5)
    for x in values:
        for y in values:
            for z in values:
                offset = np.array([x, y, z]) / 1000
                data.qpos[:] = start_q
                data.qpos[-7:-4] = center + offset
                data.ctrl[:] = start_q[:16]
                mujoco.mj_forward(model, data)
                fingers = set()
                max_penetration_mm = 0.0
                for c in data.contact:
                    a, b = model.geom_bodyid[c.geom1], model.geom_bodyid[c.geom2]
                    other = b if a == cube_body else a if b == cube_body else None
                    if other is None:
                        continue
                    max_penetration_mm = max(max_penetration_mm, -1000 * c.dist)
                    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, other)
                    prefix = name.split('_')[0]
                    if prefix in ('th', 'if', 'mf', 'rf'):
                        fingers.add(prefix)
                if len(fingers) >= 2:
                    results.append(dict(offset_mm=[float(x), float(y), float(z)],
                                        fingers=sorted(fingers),
                                        max_penetration_mm=float(max_penetration_mm)))
    clean = [r for r in results if r['max_penetration_mm'] <= 2]
    clean.sort(key=lambda r: sum(v * v for v in r['offset_mm']))
    results.sort(key=lambda r: (r['max_penetration_mm'],
                                sum(v * v for v in r['offset_mm'])))
    hold = []
    for candidate in clean:
        data.qpos[:] = start_q
        data.qpos[-7:-4] = center + np.asarray(candidate['offset_mm']) / 1000
        data.ctrl[:] = start_q[:16]
        mujoco.mj_forward(model, data)
        initial = data.qpos[-7:-4].copy()
        for _ in range(round(0.3 / model.opt.timestep)):
            mujoco.mj_step(model, data)
        fingers = set()
        for c in data.contact:
            a, b = model.geom_bodyid[c.geom1], model.geom_bodyid[c.geom2]
            other = b if a == cube_body else a if b == cube_body else None
            if other is not None:
                name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, other)
                prefix = name.split('_')[0]
                if prefix in ('th', 'if', 'mf', 'rf'):
                    fingers.add(prefix)
        hold.append(dict(offset_mm=candidate['offset_mm'],
                         cube_drift_mm=float(1000 * np.linalg.norm(data.qpos[-7:-4] - initial)),
                         final_touching_fingers=sorted(fingers)))
    report = dict(status='offline_local_two_finger_search', motor_commands_sent=0,
                  cube_side_mm=54.5, grid_offset_mm=[-10, 10, 2.5],
                  candidates_checked=len(values) ** 3,
                  two_finger_candidates=len(results),
                  two_finger_with_at_most_2mm_penetration=len(clean),
                  nearest_clean=clean[:10], least_penetrating=results[:10],
                  clean_candidate_300ms_hold=hold,
                  caveat='Local geometry search plus a short fixed-goal hold. Physical cube pose, full contact dynamics, and image-to-model registration remain unvalidated.')
    (ROOT / 'sac_multifinger_start_search_result.json').write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
