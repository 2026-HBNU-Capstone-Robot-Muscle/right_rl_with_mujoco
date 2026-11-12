"""Compare a return rotation using the physically observed grip route.

This is explicitly a reverse-direction diagnostic, not two positive90 turns.
No camera/motor APIs; the cube is never reset after the cached prefix.
"""
import json
import sys
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parent
WORK = ROOT / 'right_hand_rotation_20261109/prehardware_policy_cycle'
sys.path.insert(0, str(WORK))
import mujoco
from native_cycle import assets, restore, Guard, BOUND, ticks_to_sim, save_json, sha, angle
from whole_hand_support import sample_model
from cube_task_frame_20261105 import orientation_error_deg, face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate

def main():
    out = ROOT / 'right_hand_rotation_20261112/nominal_cycle_actual_review/support_transfer_fix_v3/observed_reverse'
    out.mkdir(exist_ok=False)
    source = ROOT / 'right_hand_rotation_20261112/stage2_work_grip/candidate.npz'
    state = ROOT / 'right_hand_rotation_20261112/stage2_work_grip/loaded_models/gain1/mu0.3_end_state.npz'
    old = dict(np.load(source, allow_pickle=False)); first = old['goals'][:1417]
    # Undo the exact observed grip transition, then undo the exact yaw path.
    reverse_grip = np.vstack([first[-1], old['goals'][1417:2334]])[-2::-1]
    reverse_yaw = np.vstack([first[644], first[645:1283]])[-2::-1]
    moving = np.vstack([reverse_grip, reverse_yaw]); hold = np.tile(moving[-1], (54, 1))
    goals = np.vstack([old['goals'], moving, hold])
    assert np.max(abs(np.diff(np.vstack([old['start'], goals]), axis=0))) <= 2
    model, initial, _, _, provenance = assets('active_ccd50')
    data = restore(model, dict(np.load(state, allow_pickle=False))); guard = Guard(model); cube = model.geom('cube').id
    begin_B = data.geom_xmat[cube].reshape(3, 3).copy(); cert = SweepCertificate(model, guard.hand, .3)
    samples = supported = clear = 0; min_upper = float('inf'); shift = drift = 0.; rows = []
    for i in range(len(old['goals']), len(goals)):
        if i == len(goals) - 54:
            C0 = data.geom_xpos[cube].copy(); B0 = data.geom_xmat[cube].reshape(3, 3).copy()
        data.ctrl[:] = ticks_to_sim(goals[i])
        for _ in range(75):
            mujoco.mj_step(model, data); stop = guard.check(data)
            if np.linalg.norm(data.qpos[-7:-4] - initial['initial_cube_center']) > .08:
                stop = list(stop) + ['cube_translation_over80mm']
            if i >= len(goals) - 54:
                C = data.geom_xpos[cube]; B = data.geom_xmat[cube].reshape(3, 3)
                lower, bad = cert.query(data, C, B); samples += 1; clear += not bad and lower > .3
                supported += sample_model(model, data, guard.hand, cube)['supported_sample']; min_upper = min(min_upper, lower)
                shift = max(shift, float(np.linalg.norm(C - C0) * 1000)); drift = max(drift, orientation_error_deg(B, B0))
            if stop: break
        if (i + 1) % 200 == 0 or stop:
            row = dict(goal=i + 1, stop=stop, relative_yaw_deg=angle(data.geom_xmat[cube].reshape(3, 3), begin_B, initial['up']))
            rows.append(row); print(json.dumps(row), flush=True)
        if stop: break
    final_B = data.geom_xmat[cube].reshape(3, 3)
    r = dict(status='observed_prefix_exact_reverse_diagnostic_not_positive180_cycle',
             prefix_goals=len(old['goals']), reverse_grip_goals=len(reverse_grip), reverse_yaw_goals=len(reverse_yaw),
             source_sha256=sha(source), cached_prefix_state_sha256=sha(state), done=i + 1, total=len(goals),
             stop=stop, final_relative_yaw_deg=angle(final_B, begin_B, initial['up']),
             final_orientation_from_initial_deg=orientation_error_deg(final_B, initial['initial_cube_rotation']),
             final_tilt_deg=face_up_error_deg(final_B, 5, initial['up']), final_hold_samples=samples,
             supported_samples=supported, upper_certified_samples=clear, minimum_upper_mm=min_upper if samples else None,
             shift_mm=shift, drift_deg=drift, worst=guard.worst, terminal_guard=guard.last, trace=rows,
             model_provenance=provenance, physical_cube_reconstructed=False, native_guards_changed=False, devices_opened=0)
    np.savez_compressed(out / 'candidate.npz', start=old['start'], goals=goals)
    np.savez_compressed(out / 'end_state.npz', **{key: getattr(data, key) for key in BOUND}, time=data.time)
    save_json(out / 'review.json', r)
    print(json.dumps({k: v for k, v in r.items() if k not in ('model_provenance', 'trace', 'terminal_guard', 'worst')}), flush=True)

if __name__ == '__main__': main()
