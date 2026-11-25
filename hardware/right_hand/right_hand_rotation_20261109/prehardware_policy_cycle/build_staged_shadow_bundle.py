"""Package first-turn stages for offline review, never for hardware output."""
from pathlib import Path
import json
import hashlib
import numpy as np

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    target=OUT/'next_cycle/staged_shadow_bundle_v2';target.mkdir(parents=True,exist_ok=True)
    review_path=OUT/'connected_sac_student_hold_nominal/review.json'
    review=json.loads(review_path.read_text())
    if not review['first_rotation_and_hold_passed'] or review['suffix_is_saved_model_teacher_not_SAC']:
        raise RuntimeError('The fresh learned-correction connection must pass first')
    old=OUT/'shadow_bundle_nominal_only';meta=json.loads((old/'bundle.json').read_text())
    sources={'actor.npz':old/'actor.npz','reference.npz':old/'reference.npz',
             'shadow_runtime.py':OUT/'shadow_runtime.py',
             'staged_shadow_runtime.py':OUT/'staged_shadow_runtime.py',
             'correction_actor.npz':OUT/'next_cycle/recovery_bc_v2_all_examples/actor.npz'}
    for name,source in sources.items():(target/name).write_bytes(source.read_bytes())
    initial=np.load(ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/initial_state.npz',allow_pickle=False)
    station=np.load(OUT/'baseline_archived_zero/end_state.npz',allow_pickle=False)['qpos'][-7:-4]
    np.savez_compressed(target/'staging.npz',up=initial['up'],common_station=station)
    connected_cases={}
    for case in ('nominal','x_plus05','friction025'):
        path=OUT/('connected_sac_student_hold_'+case)/'review.json';case_review=json.loads(path.read_text())
        if not case_review['first_rotation_and_hold_passed'] or case_review['suffix_is_saved_model_teacher_not_SAC']:
            raise RuntimeError('Each listed connected case must pass with the learned correction')
        connected_cases[case]=dict(review=str(path.relative_to(OUT)),sha256=digest(path))
    meta.update(format='first_turn_SAC_correction_position_hold_shadow_v2',
                acquisition_metric_is_global_clearance_certificate=False,
                acquisition_metric='slab_clipped_radial_screen; strict0.3mm global margin requires the separate exact cylinder support certificate',
                admissible_variant_class='AdmissibleShadowController; native bounds unchanged; model research only',
                evidence='First turn and position hold passed in3 model cases also used for teacher data; no independent generalization or second-turn claim',
                connected_model_cases=connected_cases,
                prefix_steps=6300,acquire_samples=8,correction_is_SAC=False,
                correction_is_model_teacher_behavior_clone=True,
                holding_mode='Fixed position command with native position servo; no object-feedback reentry',
                correction_history_reset='Repeat current sample into3 history slots and zero previous action at mode switch',
                first_turn_connected_review_sha256=digest(review_path),
                first_turn_model_nominal_validated=True,complete_two_turn_cycle_validated=False,
                physical_camera_registration_verified=False,hardware_execution_allowed=False,
                next_rotation_ready=False,
                model_sha256=review['provenance']['model_sha256'])
    meta['files_sha256']={name:digest(target/name) for name in list(sources)+['staging.npz']}
    (target/'bundle.json').write_text(json.dumps(meta,indent=2),encoding='utf-8')
    print(json.dumps(dict(bundle=str(target),files=len(meta['files_sha256']),hardware_execution_allowed=False)),flush=True)


if __name__=='__main__':main()
