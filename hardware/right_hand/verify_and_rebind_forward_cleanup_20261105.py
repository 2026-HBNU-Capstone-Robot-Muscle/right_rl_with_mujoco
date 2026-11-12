"""Rebind the cleanup-only change to unchanged paths and existing evidence."""
import json,hashlib,unittest
from pathlib import Path
from forward_hold_plan_20261104 import ROOT,OUT,PROOF,REQUEST,ForwardHoldPlan,hashes
from forward_hold_gate_20261104 import make_review,validate_review

def main():
    O=ROOT/'right_hand_rotation_20261105/preintegration/post_off_cleanup_fix';old=json.loads((O/'source_before/review.json').read_bytes())
    suite=unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromName(n) for n in ('test_forward_hold_20261104','test_off_settings_restore_20261105','test_verified_off_restore_context_20261105'))
    result=unittest.TextTestRunner(verbosity=1).run(suite);tests=dict(passed=result.wasSuccessful(),tests=result.testsRun,errors=[str(x[0]) for x in result.errors],failures=[str(x[0]) for x in result.failures],devices_opened=False)
    (O/'software_tests.json').write_text(json.dumps(tests,indent=2)+'\n',encoding='utf-8')
    if not result.wasSuccessful():raise RuntimeError('Software verification failed')
    p=ForwardHoldPlan(REQUEST);new=make_review(p,old['empty_recovery'],hashes())
    for key in ('plan_sha256','last_goal_ticks','start_ticks','loaded_goals','fixed_hold_samples','fixed_hold_seconds','forward_hold_models','active_motion_limit_s','goal_step_ticks','sample_period_s','current_ma','hardware_tracking_stop_rad','goal_readback_confirmation_policy','read_resynchronization_policy','sync_receive_window_policy','read_recovery_episode_policy'):
        if old[key]!=new[key]:raise RuntimeError('Cleanup patch changed motion or health scope: '+key)
    changed={k for k,v in old['code_hashes'].items() if new['code_hashes'].get(k)!=v}
    allowed={'forward_hold_plan_20261104.py','forward_hold_gate_20261104.py','leap_forward_hold_observe_20261104.py'}
    if changed!=allowed or set(new['code_hashes'])-set(old['code_hashes'])!={'verified_off_restore_context_20261105.py'}:raise RuntimeError('Unexpected source change')
    PROOF.write_text(json.dumps(new,indent=2)+'\n',encoding='utf-8');validate_review(p,PROOF)
    report=dict(status='post_OFF_cleanup_patch_validated',software=tests,source_archive=(O/'source_before').relative_to(ROOT).as_posix(),changed_sources=sorted(changed),added_source='verified_off_restore_context_20261105.py',joint_path_and_all_ON_bounds_unchanged=True,loaded_model_cases=27,hand_model_cases=9,empty_static_cases=201,empty_dynamic_cases=9,proof_sha256=hashlib.sha256(PROOF.read_bytes()).hexdigest(),no_motor_execution=True,physical_camera_failure_cleanup_test_not_yet_performed=True)
    (O/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report,indent=2))
if __name__=='__main__':main()
