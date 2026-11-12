"""Compare finger loading at the low-friction first hold and failed next turn."""
from pathlib import Path
import json
import numpy as np

from native_cycle import OUT, assets, restore, Guard, motor, save_json, sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target, orientation_error_deg


def main():
    base = OUT / 'next_cycle/handoff_refinement_20261110'
    root = base / 'robust_cycle_preparation_20261111'
    first = root / 'low_friction_first_hold_state_v4/end_state.npz'
    failed = root / 'low_friction_teacher_cycle_connection_v4/keep_teacher_pose/end_state.npz'
    folder = root / 'low_friction_next_turn_contact_diagnosis_v4'
    if folder.exists():
        raise RuntimeError('Preserve previous contact diagnosis')
    m, initial, _, _, provenance = assets('active_ccd50')
    m.geom_friction[:, 0] = .25
    cube = m.geom('cube').id
    guard = Guard(m)
    rows = []
    for label, path, yaw in (('first90_hold', first, 90.), ('failed_second180_hold', failed, 180.)):
        state = dict(np.load(path, allow_pickle=False))
        d = restore(m, state)
        B = d.geom_xmat[cube].reshape(3, 3)
        load = sample_model(m, d, guard.hand, cube)
        finger_load = {}
        for geom, value in load['by_geom'].items():
            prefix = geom.split('_')[0]
            finger_load[prefix] = finger_load.get(prefix, 0.) + value['normal_N']
        rows.append(dict(stage=label, source=path.relative_to(OUT).as_posix(), source_sha256=sha(path),
                         target_yaw_deg=yaw,
                         orientation_error_deg=orientation_error_deg(B, upright_yaw_target(initial['initial_cube_rotation'], initial['up'], yaw)),
                         loaded_model_geoms=load['loaded_hand_geoms'],
                         normal_load_by_prefix_N=finger_load,
                         supported_sample=load['supported_sample'],
                         actual_command_motor_ticks=motor(d.ctrl).tolist()))
    folder.mkdir()
    (folder / 'source_diagnose_mu025_next_turn_contact.py').write_bytes(Path(__file__).read_bytes())
    save_json(folder / 'review.json', dict(status='restored_model_contact_diagnosis_not_physical_grip_certificate',
        model_provenance=provenance, sliding_friction_hypothesis=.25, rows=rows,
        model_state_requery_only=True, motor_or_camera_access=0, hardware_execution_allowed=False))
    print(json.dumps(dict(rows=[{k:r[k] for k in ('stage','orientation_error_deg','loaded_model_geoms','normal_load_by_prefix_N')} for r in rows])),flush=True)


if __name__ == '__main__':
    main()
