"""Inspect stage-0 model landmarks against the saved camera registration; offline."""

import json
from pathlib import Path
import sys

import jax
import mujoco
import numpy as np
from mujoco_playground import registry

from hand_registration_geometry import candidate
from sac_cube_model import set_cube_physics

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'deploy'))
from leap_hw import hw_to_sim


def main():
    config = registry.get_default_config('LeapCubeFaceUp')
    config.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=config)
    state = env.reset(jax.random.PRNGKey(0))
    model = env.mj_model
    set_cube_physics(model, mass_kg=.100)
    data = mujoco.MjData(model)
    data.qpos[:] = np.asarray(state.data.qpos, dtype=float)
    loaded = json.loads((ROOT / 'sac_stage0_alignment_audit_result.json').read_text(encoding='utf-8'))
    data.qpos[:16] = hw_to_sim(np.asarray(loaded['loaded_actual_ticks']) * 2 * np.pi / 4096)
    mujoco.mj_forward(model, data)
    registration = json.loads((ROOT / 'sac_registration_anchor_result.json').read_text(encoding='utf-8'))
    transform = candidate()
    R = np.asarray(transform['hand_to_world_candidate'])
    grasp = np.array([.11, 0, .03])
    marker = grasp - R @ np.asarray(transform['grasp_site_in_marker_m_candidate'])
    cube = np.asarray(registration['cube_center_in_sim_world_candidate_m'])
    data.qpos[-7:-4] = cube
    data.qpos[-4:] = registration['cube_quat_world_candidate']
    mujoco.mj_forward(model, data)
    cube_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    contacts = []
    for contact in data.contact:
        if cube_geom_id not in (int(contact.geom1), int(contact.geom2)):
            continue
        other = int(contact.geom2 if int(contact.geom1) == cube_geom_id else contact.geom1)
        contacts.append({'geom': mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, other),
                         'penetration_mm': float(-1000 * contact.dist),
                         'point_world_m': contact.pos.tolist()})
    joints = {}
    joint_ranges = {}
    model_joint_order = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
                         for i in range(min(model.njnt, 16))]
    for name in ('if_mcp', 'mf_mcp', 'rf_mcp', 'th_cmc', 'if_pip', 'mf_pip', 'rf_pip'):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        joints[name] = data.xanchor[joint_id].tolist()
        address = model.jnt_qposadr[joint_id]
        joint_ranges[name] = {'position_rad': float(data.qpos[address]),
                              'range_rad': model.jnt_range[joint_id].tolist(),
                              'limited': bool(model.jnt_limited[joint_id])}
    geoms = {}
    cube_to_finger = []
    for i in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or ''
        if name.startswith(('if_', 'mf_', 'rf_')) and 'collision' in name:
            geoms[name] = {'center': data.geom_xpos[i].tolist(),
                           'half_extent_or_radius': model.geom_size[i].tolist(),
                           'type': int(model.geom_type[i])}
            fromto = np.zeros(6)
            separation = mujoco.mj_geomDistance(model, data, cube_geom_id, i, 1., fromto)
            cube_to_finger.append({'geom': name, 'separation_mm': float(1000 * separation)})
    cube_to_finger.sort(key=lambda item: item['separation_mm'])
    result = {'candidate_marker_center_world_m': marker.tolist(),
              'candidate_cube_center_world_m': cube.tolist(),
              'candidate_cube_minus_marker_world_m': (cube - marker).tolist(),
              'candidate_marker_normal_world': R[:, 2].tolist(),
              'model_joint_axes_world_m': joints,
              'model_joint_order_first16': model_joint_order,
              'model_joint_ranges': joint_ranges,
              'cube_contact_geoms': contacts,
              'nearest_finger_geoms': cube_to_finger[:12],
              'cube_geom_friction': model.geom_friction[cube_geom_id].tolist(),
              'palm_collision_friction': {
                  name: model.geom_friction[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)].tolist()
                  for name in ('palm_collision_6', 'palm_collision_7', 'palm_collision_8')},
              'model_finger_collision_geoms': geoms,
              'hardware_authorized': False}
    (ROOT / 'sac_stage0_geometry_audit_result.json').write_text(
        json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({k: v for k, v in result.items() if k != 'model_finger_collision_geoms'}, indent=2))


if __name__ == '__main__':
    main()
