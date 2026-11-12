"""Recover the actual low-friction first-hold state for offline next-turn planning."""
from pathlib import Path
import json
import numpy as np
import mujoco

from native_cycle import OUT, assets, restore, Guard, BOUND, motor, ticks_to_sim, save_json, sha
from cube_task_frame_20261105 import upright_yaw_target, orientation_error_deg


def main():
    base = OUT / 'next_cycle/handoff_refinement_20261110'
    source = base / 'robust_cycle_preparation_20261111/low_friction_teacher_cycle_connection_v4/keep_teacher_pose'
    root = base / 'robust_cycle_preparation_20261111/low_friction_first_hold_state_v4'
    if root.exists():
        raise RuntimeError('Preserve previous extracted state')
    review = json.loads((source / 'review.json').read_text(encoding='utf-8'))
    assert review['first_hold_passed'] and review['holds'][0]['samples'] == 4050
    seam = review['seams'][0]['after_goal']
    count = seam + review['holds'][0]['samples'] // 75
    goal_path = source / 'research_goals.npz'
    z = np.load(goal_path, allow_pickle=False)
    goals = z['goals'][:count]
    model, initial, _, _, provenance = assets('active_ccd50')
    model.geom_friction[:, 0] = .25
    data = restore(model, initial)
    guard = Guard(model)
    previous = motor(data.ctrl)
    for i, command in enumerate(goals):
        assert np.max(np.abs(command - previous)) <= 2 + 1e-9
        assert np.all(command >= guard.lo) and np.all(command <= guard.hi)
        data.ctrl[:] = ticks_to_sim(command)
        previous = command.copy()
        for _ in range(75):
            mujoco.mj_step(model, data)
            reasons = guard.check(data)
            assert not reasons, (i, reasons)
            assert np.linalg.norm(data.qpos[-7:-4] - initial['initial_cube_center']) <= .08
    trajectory = np.load(source / 'trajectory.npz', allow_pickle=False)['qpos']
    error = float(np.max(np.abs(data.qpos - trajectory[count])))
    assert error == 0., error
    cube = model.geom('cube').id
    pose = data.geom_xmat[cube].reshape(3, 3)
    first_error = orientation_error_deg(pose, upright_yaw_target(initial['initial_cube_rotation'], initial['up'], 90.))
    assert abs(first_error - review['holds'][0]['orientation_error_deg']) < 1e-8
    root.mkdir()
    (root / 'source_extract_mu025_first_hold_state.py').write_bytes(Path(__file__).read_bytes())
    np.savez_compressed(root / 'end_state.npz', **{key: getattr(data, key) for key in BOUND}, time=data.time,
                        up=initial['up'], initial_cube_center=initial['initial_cube_center'],
                        initial_cube_rotation=initial['initial_cube_rotation'])
    save_json(root / 'review.json', dict(status='exact_actual_low_friction_first_hold_state',
        source=goal_path.relative_to(OUT).as_posix(), source_sha256=sha(goal_path),
        parent_review_sha256=sha(source / 'review.json'), model_provenance=provenance,
        sliding_friction_hypothesis=.25, replayed_goal_count=count, teacher_goal_count=seam,
        first_hold_goal_count=count-seam, trajectory_qpos_difference=error,
        first90_error_deg=first_error, actual_state_resets_after_initial_placement=0,
        original_native_guards_and_joint_limits_preserved=True, devices_opened=0,
        hardware_execution_allowed=False))
    print(json.dumps(dict(replayed_goal_count=count, trajectory_qpos_difference=error,
                          first90_error_deg=first_error)), flush=True)


if __name__ == '__main__':
    main()
