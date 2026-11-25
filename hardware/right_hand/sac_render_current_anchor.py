"""Render the saved real-camera start in the simulation; offline only."""

import json
from pathlib import Path

import jax
import mujoco
import numpy as np
from mujoco_playground import registry

from sac_cube_model import set_cube_physics
from sac_render_geometry import render
from leap_hw import hw_to_sim

ROOT = Path(__file__).resolve().parent


def main():
    config = registry.get_default_config('LeapCubeFaceUp')
    config.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=config)
    state = env.reset(jax.random.PRNGKey(0))
    model = env.mj_model
    set_cube_physics(model, mass_kg=.100)
    cube_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    model.geom_rgba[cube_geom] = [1., .25, .1, 1.]
    qpos = np.asarray(state.data.qpos, dtype=float).copy()
    raw = json.loads((ROOT / 'sequence_flip_poses.json').read_text(encoding='utf-8'))['poses'][0]['raw_ticks']
    qpos[:16] = hw_to_sim(np.asarray(raw, dtype=float) * 2 * np.pi / 4096)
    observed = json.loads((ROOT / 'sac_registration_anchor_result.json').read_text(encoding='utf-8'))
    qpos[-7:-4] = observed['cube_center_in_sim_world_candidate_m']
    qpos[-4:] = observed['cube_quat_world_candidate']
    render(model, qpos, ROOT / 'sac_current_anchor_collision.png', True)
    fit = json.loads((ROOT / 'sac_fit_contact_real_cube_100g_candidate_diagnostic_result.json').read_text(encoding='utf-8'))
    if fit['contact_retained_top_candidates']:
        qpos[-7:-4] = fit['contact_retained_top_candidates'][0]['start_m']
        render(model, qpos, ROOT / 'sac_current_contact_candidate.png', True)
    print('Saved offline render; no device I/O')


if __name__ == '__main__':
    main()
