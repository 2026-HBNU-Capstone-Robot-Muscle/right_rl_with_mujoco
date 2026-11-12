"""Reassess saved evidence using NumPy only; never open devices or start WSL."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
import hashlib
import json
import argparse
import re
import numpy as np
from whole_hand_support import CONTRACT,summarize_contacts

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def read(path):return json.loads(path.read_text(encoding='utf-8'))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output-tag',default='whole_hand_preflight_v2')
    args=parser.parse_args()
    if not re.fullmatch('[a-z0-9_]+',args.output_tag):raise ValueError('Use a simple local artifact tag')
    FOLDER=OUT/'next_cycle/handoff_refinement_20261110'/args.output_tag
    if FOLDER.exists():raise RuntimeError('Preserve completed preflight preparation')
    audit_path=FOLDER.parent/'support_audit.json';audit=read(audit_path)
    cached=[];gravity_up=None
    for row in audit['states']:
        source=OUT/row['source']
        if sha(source)!=row['source_sha256']:raise AssertionError('Saved state changed')
        wrench=row['wrench'];force=np.array(wrench['force_world_N']);net=np.array(wrench['net_force_with_gravity_N'])
        up=(force-net)/wrench['weight_N']
        np.testing.assert_allclose(np.linalg.norm(up),1.,atol=1e-10)
        if gravity_up is None:gravity_up=up
        else:np.testing.assert_allclose(up,gravity_up,atol=1e-10)
        contacts=[dict(geom=name,other_body_id=1,**value) for name,value in wrench['by_geom'].items()]
        sample=summarize_contacts(contacts,{1},-up,wrench['weight_N'])
        np.testing.assert_allclose(sample['hand_force_world_N'],force,atol=1e-12)
        np.testing.assert_allclose(sample['upward_support_N'],wrench['upward_support_N'],atol=1e-12)
        cached.append(dict(source=row['source'],source_sha256=row['source_sha256'],
            model_kind=audit['model_provenance']['model_kind'],
            load_accounting=sample,
            external_contact_data_available=False,
            conditional_contact_accounting_only_not_a_new_hold_pass=True,
            existing_constraint_force_error=wrench['constraint_force_error']))
    trials=[]
    for path in sorted(OUT.glob('next_cycle/staged_model_*/review.json')):
        r=read(path)
        if r.get('status')!='packaged_controller_free_model_trial':continue
        other=[reason for reason in r['reasons'] if reason!='lower_contacts']
        row=dict(path=path.relative_to(OUT).as_posix(),sha256=sha(path),steps=r['steps'],
                 model_kind=r.get('model_kind','active'),original_passed=r['first_rotation_and_hold_passed'],
                 original_reasons=r['reasons'],stop=r.get('stop'),
                 remaining_reasons_if_legacy_tip_pair_removed=other,
                 only_legacy_contact_reason_might_change=bool(r['reasons'] and not other and not r.get('stop')),
                 old_result_reclassified=False)
        if r.get('model_kind')=='active_ccd50' and r.get('stop'):
            state=path.parent/'end_state.npz'
            z=np.load(state,allow_pickle=False)
            center=z['qpos'][-7:-4];delta=center-z['initial_cube_center']
            # These stored models have16 hinge velocities followed by a cube free joint.
            if len(z['qvel'])!=22:raise AssertionError('Unexpected cube velocity layout')
            velocity=z['qvel'][16:19];vertical=float(delta@gravity_up)
            row['stored_endpoint_kinematics']=dict(
                state_sha256=sha(state),relative_to_initial_center_mm=(delta*1000).tolist(),
                displacement_mm=float(np.linalg.norm(delta)*1000),upward_mm=vertical*1000,
                lateral_mm=float(np.linalg.norm(delta-vertical*gravity_up)*1000),
                cube_linear_velocity_world_m_s=velocity.tolist(),upward_velocity_m_s=float(velocity@gravity_up),
                hand_contact_forces_at_stop_not_saved=True,
                complete_hand_contact_loss_or_outside_hand_not_established_by_this_analysis=True)
        trials.append(row)
    FOLDER.mkdir()
    report=dict(status='whole_hand_evaluator_prepared_and_saved_evidence_reprocessed',
        date='2026-11-10',support_contract=CONTRACT,
        source_support_audit_sha256=sha(audit_path),
        cached_model_force_checks=cached,archived_trial_review=trials,
        archived_failures_only_due_to_legacy_tip_contacts=sum(row['only_legacy_contact_reason_might_change'] for row in trials),
        fresh_native_model_evaluations_executed=False,
        exact_recorded_ring_geometry_evaluation_executed=False,
        native_execution_blocker=dict(action='WSL native model comparison',
            reason='Automatic approval review usage limit; prior requested action was not executed',
            is_unsafe_action_determination=False,approval_bypass_attempted=False),
        unchanged_hardware_contract=dict(current_mA=450,max_goal_increment_ticks=2,period_seconds=.075,
            existing_PID_and_native_guards_preserved=True),
        first_rotation_and_hold_passed_on_new_whole_hand_criterion=False,
        next_rotation_passed=False,whole_cycle_ready_for_hardware=False,
        hardware_execution_allowed=False,devices_opened=0,remote_writes=0,monitoring_ui_changes=0,
        files_sha256={name:sha(OUT/name) for name in (
            'whole_hand_support.py','test_whole_hand_support.py','evaluate_staged_shadow_model.py',
            'audit_recorded_ring_pose.py','prepare_whole_hand_preflight.py')})
    (FOLDER/'review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    (FOLDER/'source_prepare_whole_hand_preflight.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps(dict(status=report['status'],cached_model_force_checks=len(cached),
        archived_model_trials=len(trials),legacy_contact_only_failures=report['archived_failures_only_due_to_legacy_tip_contacts'],
        CCD50_stops=[dict(path=row['path'],stop=row['stop'],kinematics=row.get('stored_endpoint_kinematics'))
                    for row in trials if row['model_kind']=='active_ccd50' and row['stop']],
        hardware_execution_allowed=False),ensure_ascii=False),flush=True)


if __name__=='__main__':main()
