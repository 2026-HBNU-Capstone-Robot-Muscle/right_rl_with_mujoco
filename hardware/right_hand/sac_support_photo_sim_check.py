"""Offline compare prior registered pose with the later qualitative side photo."""
import argparse
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


def hold(model, base_q, ticks, placement, *, lock_hand=False):
    q = base_q.copy()
    q[:16] = hw_to_sim(np.asarray(ticks, dtype=float)*2*np.pi/4096)
    q[-7:-4] = placement['cube_center_in_sim_world_candidate_m']
    q[-4:] = placement['cube_quat_world_candidate']
    data = mujoco.MjData(model)
    data.qpos[:] = q
    data.ctrl[:] = q[:16]
    cube_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'cube')
    mujoco.mj_forward(model, data)
    def contacts():
        found = []
        for item in data.contact:
            a, b = model.geom_bodyid[item.geom1], model.geom_bodyid[item.geom2]
            if cube_body in (a, b):
                other = item.geom2 if a == cube_body else item.geom1
                found.append((float(item.dist), mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, other)))
        return found
    initial = contacts()
    for _ in range(30):
        if lock_hand:
            data.qpos[:16] = q[:16]
            data.qvel[:16] = 0
            mujoco.mj_forward(model, data)
        mujoco.mj_step(model, data)
    final = contacts()
    return {'initial_contacts': len(initial),
            'initial_geoms': sorted({item[1] for item in initial}),
            'initial_penetration_mm': -1000*min([0.]+[item[0] for item in initial]),
            'final_contacts': len(final),
            'final_geoms': sorted({item[1] for item in final}),
            'drift_mm': float(1000*np.linalg.norm(data.qpos[-7:-4]-q[-7:-4]))}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--registration-file', default='sac_registration_support_photo_diagnostic.json')
    parser.add_argument('--output-file', default='sac_support_photo_sim_check_result.json')
    parser.add_argument('--trial-file', default='leap_support_capture_result.json')
    args = parser.parse_args()
    old = json.loads((ROOT / 'sac_registration_anchor_result.json').read_text(encoding='utf-8'))
    new = json.loads((ROOT / args.registration_file).read_text(encoding='utf-8'))
    trial = json.loads((ROOT / args.trial_file).read_text(encoding='utf-8'))
    if trial.get('status') != 'support_photo_complete' or not trial.get('all_torque_off_verified'):
        raise RuntimeError('Completed support photo with torque OFF verification required')
    vision_folder_name = trial['vision_recording']['folder'].replace('\\', '/').split('/')[-1]
    vision = json.loads((ROOT / vision_folder_name / 'result.json').read_text(encoding='utf-8'))
    capture_time = trial['cube_placement_camera_snapshot']['captured_at_unix_s']
    vision_row = min(vision['frames'], key=lambda row: abs(row['captured_at_unix_s']-capture_time))
    motor_row = min((row for row in trial['samples'] if row['phase']=='place_cube'),
                    key=lambda row: abs(row['t']-vision_row['motor_t_s']))
    old_ticks = json.loads((ROOT / 'sac_stage0_alignment_audit_result.json').read_text(encoding='utf-8'))['loaded_actual_ticks']
    new_ticks = motor_row['actual']
    config = registry.get_default_config('LeapCubeFaceUp')
    config.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=config)
    model = env.mj_model
    set_cube_physics(model, mass_kg=.100)
    state = env.reset(jax.random.PRNGKey(0))
    base_q = np.asarray(state.data.qpos, dtype=float)
    old_q = np.asarray(old['cube_quat_world_candidate'], dtype=float)
    new_q = np.asarray(new['cube_quat_world_candidate'], dtype=float)
    angle = np.degrees(2*np.arccos(np.clip(abs(float(old_q@new_q)), -1, 1)))
    result = {'cube_center_shift_mm': (1000*(np.asarray(new['cube_center_in_sim_world_candidate_m'])-
                                           np.asarray(old['cube_center_in_sim_world_candidate_m']))).tolist(),
              'cube_orientation_difference_deg': float(angle),
              'motor_sample_offset_s': float(motor_row['t']-vision_row['motor_t_s']),
              'max_motor_tick_change_from_old': int(np.max(np.abs(np.asarray(new_ticks)-np.asarray(old_ticks)))),
              'previous': hold(model, base_q, old_ticks, old),
              'previous_rigid_joint_diagnostic': hold(model, base_q, old_ticks, old, lock_hand=True),
              'support_photo_diagnostic': hold(model, base_q, new_ticks, new),
              'support_photo_rigid_joint_diagnostic': hold(model, base_q, new_ticks, new, lock_hand=True),
              'registration_verified': False,
              'hardware_authorized': False}
    (ROOT / args.output_file).write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
