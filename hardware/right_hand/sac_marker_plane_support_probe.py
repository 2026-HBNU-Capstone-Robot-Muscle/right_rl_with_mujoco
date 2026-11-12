"""Test one ID40-referenced palm shelf across independent saved first-pose runs.

This is an offline sensitivity test. It does not edit the MJCF or drive motors.
"""

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
sys.path.insert(0, str(ROOT.parent/'work/Leap_Test/deploy'))
from leap_hw import hw_to_sim

RUNS = [
    ('oct9_2216', 'sac_cube_rim_register_20261009_result.json',
     'leap_sequence_flip_20261009_221628_602843.json'),
    ('oct9_2316', 'sac_cube_rim_register_2316_result.json',
     'leap_sequence_flip_20261009_231632_206951.json'),
    ('oct10_0459', 'sac_cube_rim_register_1010_stage0_result.json',
     'leap_support_capture_result.json'),
]


def mat_to_quat(R):
    q = np.empty(4)
    mujoco.mju_mat2Quat(q, np.ascontiguousarray(R.reshape(-1)))
    return q


def main():
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=cfg)
    model = env.mj_model
    set_cube_physics(model, mass_kg=.100)
    cube_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'palm')
    shelf_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'palm_collision_10')
    reference = candidate()
    R_wm = np.asarray(reference['hand_to_world_candidate'])
    grasp_in_marker = np.asarray(reference['grasp_site_in_marker_m_candidate'])
    marker_world = np.array([.11, 0, .03]) - R_wm @ grasp_in_marker
    box_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM,
                                  f'palm_collision_{i}') for i in (6,7,8)]
    shelf_pos_original = model.geom_pos[shelf_id].copy()
    shelf_quat_original = model.geom_quat[shelf_id].copy()
    shelf_size_original = model.geom_size[shelf_id].copy()
    box_types = [(int(model.geom_contype[i]), int(model.geom_conaffinity[i])) for i in box_ids]
    cases = []
    for label, reg_file, trial_file in RUNS:
        model.geom_pos[shelf_id] = shelf_pos_original
        model.geom_quat[shelf_id] = shelf_quat_original
        model.geom_size[shelf_id] = shelf_size_original
        for geom_id, (contype, conaffinity) in zip(box_ids, box_types):
            model.geom_contype[geom_id] = contype
            model.geom_conaffinity[geom_id] = conaffinity
        mujoco.mj_setConst(model, mujoco.MjData(model))
        reg = json.loads((ROOT/reg_file).read_text())
        trial = json.loads((ROOT/trial_file).read_text())
        samples = [s for s in trial['samples'] if s['phase']=='place_cube']
        if not samples:
            raise RuntimeError(f'{label}: no place_cube samples')
        q = np.asarray(env.reset(jax.random.PRNGKey(0)).data.qpos, dtype=float)
        q[:16] = hw_to_sim(np.asarray(samples[-1]['actual'])*2*np.pi/4096)
        q[-7:-4] = reg['cube_center_world_m']
        q[-4:] = reg['cube_quat_world_candidate']
        baseline = probe(model, q, cube_id)
        placement = mujoco.MjData(model)
        placement.qpos[:] = q
        mujoco.mj_forward(model, placement)
        R_wp = np.asarray(placement.xmat[palm_id]).reshape(3,3)
        p_wp = np.asarray(placement.xpos[palm_id])
        for surface_mm in (-2., 0., 2.):
            for xy_mm in ((0.,0.), (14.,11.)):
                # Fixed ID40 plane/XY offsets, reused across all runs.
                center_marker = np.array([xy_mm[0], xy_mm[1], surface_mm-2.])/1000
                center_world = marker_world + R_wm @ center_marker
                model.geom_pos[shelf_id] = R_wp.T @ (center_world-p_wp)
                model.geom_quat[shelf_id] = mat_to_quat(R_wp.T @ R_wm)
                model.geom_size[shelf_id,:3] = [.035,.035,.002]
                for geom_id in box_ids:
                    model.geom_contype[geom_id] = 0
                    model.geom_conaffinity[geom_id] = 0
                mujoco.mj_setConst(model, mujoco.MjData(model))
                result = probe(model, q, cube_id)
                cases.append({'run':label,'surface_from_marker_mm':surface_mm,
                              'shelf_xy_from_marker_mm':xy_mm,
                              'baseline_drift_mm':baseline['drift_mm'],**result})
    report = {'status':'offline_marker_plane_support_probe','hardware_authorized':False,
              'marker_world_m':marker_world.tolist(),
              'runs':len(RUNS),'cases':cases,
              'caveat':'Marker-to-model transform and palm surface offset are provisional; one fixed synthetic plate is not an independently measured collision model.'}
    (ROOT/'sac_marker_plane_support_probe_result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'status':report['status'],'runs':len(RUNS),
                      'cases':[{'run':r['run'],'height':r['surface_from_marker_mm'],
                                'xy':r['shelf_xy_from_marker_mm'],
                                'penetration_mm':r['penetration_mm'],
                                'drift_mm':r['drift_mm']} for r in cases]},indent=2))


if __name__=='__main__':
    main()
