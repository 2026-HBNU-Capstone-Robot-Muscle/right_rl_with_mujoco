"""Offline hold check along the camera ray of the measured one-marker pose."""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'deploy'))
from leap_hw import hw_to_sim


def camera_ray_world():
    import cv2
    from hand_registration_geometry import candidate
    from sac_registration_from_anchor import detect, marker_pose
    folder = ROOT / 'sequence_vision_20261006_203648_332236'
    with np.load(ROOT / 'camera1_720.npz') as camera:
        K, dist = camera['K'].copy(), camera['dist'].copy()
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    _, e_corners, e_ids, _ = detect(folder / 'anchor_empty_stage0.png', dictionary)
    _, p_corners, p_ids, _ = detect(folder / 'placement_at_space.png', dictionary)
    R_h0, _, _ = marker_pose(e_corners, e_ids, 40, .0282, K, dist)
    R_a0, _, _ = marker_pose(e_corners, e_ids, 41, .030, K, dist)
    R_a1, _, _ = marker_pose(p_corners, p_ids, 41, .030, K, dist)
    R_wm = np.asarray(candidate()['hand_to_world_candidate'])
    ray = R_wm @ R_h0.T @ R_a0 @ R_a1.T @ np.array([0., 0., 1.])
    return ray / np.linalg.norm(ray)


def main():
    if '--write-ray' in sys.argv:
        ray = camera_ray_world()
        (ROOT / 'sac_depth_sensitivity_ray.json').write_text(json.dumps(ray.tolist())+'\n', encoding='utf-8')
        print(json.dumps(ray.tolist()))
        return
    import jax
    import mujoco
    from mujoco_playground import registry
    from sac_cube_model import set_cube_physics
    sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'deploy'))
    from leap_hw import hw_to_sim
    config = registry.get_default_config('LeapCubeFaceUp')
    config.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=config)
    model = env.mj_model
    set_cube_physics(model, mass_kg=.100)
    state = env.reset(jax.random.PRNGKey(0))
    q = np.asarray(state.data.qpos, dtype=float).copy()
    audit = json.loads((ROOT / 'sac_stage0_alignment_audit_result.json').read_text(encoding='utf-8'))
    q[:16] = hw_to_sim(np.asarray(audit['loaded_actual_ticks']) * 2 * np.pi / 4096)
    placement = json.loads((ROOT / 'sac_registration_anchor_result.json').read_text(encoding='utf-8'))
    center = np.asarray(placement['cube_center_in_sim_world_candidate_m'], dtype=float)
    q[-4:] = placement['cube_quat_world_candidate']
    ray = np.asarray(json.loads((ROOT / 'sac_depth_sensitivity_ray.json').read_text(encoding='utf-8')))
    cube_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'cube')
    rows = []
    for offset_mm in range(-30, 31, 5):
        q[-7:-4] = center + ray * (offset_mm/1000)
        data = mujoco.MjData(model)
        data.qpos[:] = q
        data.ctrl[:] = q[:16]
        mujoco.mj_forward(model, data)
        def contacts():
            out = []
            for contact in data.contact:
                a, b = model.geom_bodyid[contact.geom1], model.geom_bodyid[contact.geom2]
                if cube_body in (a, b):
                    out.append(float(contact.dist))
            return out
        initial = contacts()
        for _ in range(30):
            mujoco.mj_step(model, data)
        final = contacts()
        rows.append({'camera_ray_offset_mm': offset_mm,
                     'initial_penetration_mm': -1000*min([0.]+initial),
                     'initial_contacts': len(initial), 'final_contacts': len(final),
                     'drift_mm': float(1000*np.linalg.norm(data.qpos[-7:-4]-q[-7:-4]))})
    result = {'camera_ray_world': ray.tolist(), 'rows': rows,
              'hardware_authorized': False}
    (ROOT / 'sac_depth_sensitivity_result.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
