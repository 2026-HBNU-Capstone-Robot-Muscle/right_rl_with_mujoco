"""Bounded contact-position comparison from a saved free-cube checkpoint.

These local probes do not stand in for the complete placement matrix. No motor
program is imported or executed, and no force/gain/collision bound is relaxed.
"""
import os
os.environ['JAX_PLATFORMS'] = 'cpu'
import argparse
import hashlib
import json
import time
import numpy as np
import mujoco
from dynamic_common_grip_search_20261105 import CP, ROOT, R, ramp
from research_rotation_body_hold_20261104 import motor, inspect_for, native
from canonical_grip_dynamics_20261105 import run
from canonical_grip_task_review_20261105 import task_reasons
from evaluate_canonical_grip_reaction_20261105 import evaluate
from cube_contact_material_policy_20261029 import apply_material


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--middle', type=int, choices=(-8, 0, 8), required=True)
    a = ap.parse_args()
    out = ROOT / f'strength_refinement/m{a.middle:+d}'
    out.mkdir(parents=True, exist_ok=True)
    m = mujoco.MjModel.from_binary_path(str(CP / 'end_model.mjb'))
    z = np.load(CP / 'end_state.npz')
    base = np.load(ROOT / 'contact_preserving_free/nominal/end_g1_f0.3.npz')
    old = np.load(R / 'right_hand_rotation_20261104/forward_hold/candidate.npz')
    start = motor(z['ctrl'])
    if not np.array_equal(start, old['goals'][-1]):
        raise ValueError('checkpoint does not match retained forward prefix')
    hand, _, _ = inspect_for(m)
    apply_material(m, .3, hand, m.geom('cube').id)
    lo, hi, _, _ = native(m)
    cases = []
    for thumb in (-56, -48, -40):
        began = time.perf_counter()
        folder = out / f't{thumb:+d}'
        folder.mkdir(exist_ok=True)
        goal = motor(base['ctrl'])
        goal[7] -= 77
        goal[11] += a.middle
        goal[3] += thumb
        if np.any(goal < lo) or np.any(goal > hi):
            raise ValueError('native goal range')
        rows = ramp(start, goal, 134)
        if np.max(np.abs(np.diff(np.vstack([start, rows]), axis=0))) > 2:
            raise ValueError('2tick goal increment')
        end = folder / 'end_state.npz'
        dynamic = run(m, z, rows, 1, end, None, True)
        why = task_reasons(dynamic)
        loads = []
        if not why:
            final = np.load(end)
            for torque in (-.01, .01):
                loads.append(evaluate(m, final, torque, folder / f'load{torque:+g}.npz'))
        accepted = not why and len(loads) == 2 and all(c['load_filter_passed'] for c in loads)
        candidate = folder / 'candidate.npz'
        np.savez_compressed(candidate, start=old['start'],
                            goals=np.vstack([old['goals'], rows]),
                            phases=np.r_[old['phases'], ['postturn_body_regrasp'] * (len(rows)-134), ['postturn_body_hold'] * 134])
        row = dict(middle_ticks=a.middle, thumb_ticks=thumb, dynamic=dynamic,
                   suffix_reasons=why, hypothetical_reaction=loads,
                   local_filter_passed=accepted, candidate=str(candidate.relative_to(R)),
                   candidate_sha256=hashlib.sha256(candidate.read_bytes()).hexdigest(),
                   elapsed_seconds=time.perf_counter()-began)
        cases.append(row)
        (out / 'review.json').write_text(json.dumps(dict(
            status='free_cube_local_contact_strength_comparison', cases=cases,
            checkpoint=str((CP/'end_state.npz').relative_to(R)),
            full_placement_prefix_not_replayed=True, gain_scale=1., friction=.3,
            initial_cube_state_not_reset_within_probe=True,
            physical_actual_load_not_measured=True,
            hardware_execution_allowed=False, devices_opened=False), indent=2)+'\n', encoding='utf-8')
        print(json.dumps(dict(middle=a.middle, thumb=thumb, suffix_reasons=why,
                              regrasp_yaw=dynamic['regrasp_yaw_change_deg'],
                              gap=dynamic['settled_tail']['minimum_separating_plane_gap_1ms_mm'],
                              loads=[dict(torque=c['hypothetical_reaction_torque_nm'], passed=c['load_filter_passed'],
                                          yaw=c['max_yaw_change_deg'], reasons=c['reasons']) for c in loads],
                              accepted=accepted, elapsed_seconds=row['elapsed_seconds'])), flush=True)


if __name__ == '__main__':
    main()
