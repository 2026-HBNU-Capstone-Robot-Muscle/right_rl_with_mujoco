"""Diagnostic: can a cube-sized palm support missing from coarse collision boxes explain drift?

Repositions one existing box in memory only. The added shelf is a hypothesis,
not measured geometry and must not be used for policy training as-is.
"""

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
sys.path.insert(0, str(ROOT.parent/'work/Leap_Test/deploy'))
from leap_hw import hw_to_sim

def mat_to_quat(R):
    q = np.empty(4)
    mujoco.mju_mat2Quat(q, np.ascontiguousarray(R.reshape(-1)))
    return q


def main():
    reg = json.loads((ROOT/'sac_cube_rim_register_1010_stage0_result.json').read_text())
    trial = json.loads((ROOT/'leap_support_capture_result.json').read_text())
    actual = [s for s in trial['samples'] if s['phase']=='place_cube'][-1]['actual']
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=cfg)
    model = env.mj_model
    set_cube_physics(model, mass_kg=.100)
    cube_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'palm')
    shelf_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'palm_collision_10')
    q = np.asarray(env.reset(jax.random.PRNGKey(0)).data.qpos, dtype=float)
    q[:16] = hw_to_sim(np.asarray(actual)*2*np.pi/4096)
    q[-7:-4] = reg['cube_center_world_m']
    q[-4:] = reg['cube_quat_world_candidate']
    baseline = probe(model, q, cube_id)
    data = mujoco.MjData(model)
    data.qpos[:] = q
    mujoco.mj_forward(model, data)
    R_wp = np.asarray(data.xmat[palm_id]).reshape(3,3)
    p_wp = np.asarray(data.xpos[palm_id])
    R_wc = np.zeros((3,3))
    mujoco.mju_quat2Mat(R_wc.reshape(-1), q[-4:])
    # Cube face 5 is green-up in the photographed start.
    u, v, n = np.array([0.,0.,-1.]), np.array([1.,0.,0.]), np.array([0.,-1.,0.])
    R_plate = R_wc @ np.column_stack((u,v,n))
    normal = R_plate[:,2]
    center_world = q[-7:-4] - normal*(.02725+.002)
    model.geom_pos[shelf_id] = R_wp.T @ (center_world-p_wp)
    model.geom_quat[shelf_id] = mat_to_quat(R_wp.T @ R_plate)
    model.geom_size[shelf_id,:3] = [.031, .031, .002]
    mujoco.mj_setConst(model, mujoco.MjData(model))
    check = mujoco.MjData(model)
    check.qpos[:] = q
    mujoco.mj_forward(model, check)
    shelf_distance = float(mujoco.mj_geomDistance(model, check, cube_id, shelf_id, 1.0, None))
    supported = probe(model, q, cube_id)
    for index in (6, 7, 8):
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM,
                                    f'palm_collision_{index}')
        model.geom_contype[geom_id] = 0
        model.geom_conaffinity[geom_id] = 0
    replacement = probe(model, q, cube_id)
    report = {'status':'offline_synthetic_support_hypothesis','hardware_authorized':False,
              'baseline':baseline,'synthetic_shelf':supported,
              'shelf_replacing_palm_boxes_6_7_8': replacement,
              'shelf_center_world_m':center_world.tolist(),
              'shelf_cube_signed_distance_mm': shelf_distance*1000,
              'shelf_contype': int(model.geom_contype[shelf_id]),
              'shelf_conaffinity': int(model.geom_conaffinity[shelf_id]),
              'limitation':'Shelf is placed from the cube pose itself, not a measured palm surface; diagnostic only.'}
    (ROOT/'sac_flat_support_probe_1010_result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
