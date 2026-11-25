"""Offline-only research queue; --check requires no MuJoCo, network or devices.

Native mode must be invoked in an already approved MuJoCo environment. This
script never launches WSL, changes permissions, installs dependencies or drives
the robot. Existing results are reused only if their source snapshots match.
"""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
import argparse
import hashlib
import json
import runpy

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
FOLDER=OUT/'next_cycle/handoff_refinement_20261110/whole_hand_preflight_v2'
TASKS=[
    dict(name='exact_recorded_ring_geometry',script='audit_recorded_ring_pose.py',args=[],
         output='next_cycle/handoff_refinement_20261110/recorded_ring_reassessment/geometry_audit.json',
         snapshot='source_audit_recorded_ring_pose.py'),
    dict(name='saved_state_whole_hand_contacts',script='audit_whole_hand_saved_states.py',args=[],
         output='next_cycle/handoff_refinement_20261110/whole_hand_saved_state_audit/review.json',
         snapshot='source_audit_whole_hand_saved_states.py'),
    dict(name='fresh_first_rotation_whole_hand_support',script='evaluate_staged_shadow_model.py',
         args=['--model-kind','active_ccd50','--case','nominal','--integer-encoders',
               '--admissible-correction','--hold-extra-steps','400','--support-mode','whole_hand_v1'],
         output='next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_whole_hand_v1_active_ccd50/review.json',
         snapshot='source_evaluate_staged_shadow_model.py')]


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def read(path):return json.loads(path.read_text(encoding='utf-8'))


def check():
    report=read(FOLDER/'review.json')
    revisions={}
    for name,digest in report['files_sha256'].items():
        if sha(OUT/name)!=digest:
            if name!='evaluate_staged_shadow_model.py':raise RuntimeError('Prepared source changed: '+name)
            from evaluator_revision_evidence import validate_evaluator_revision
            revisions[name]=validate_evaluator_revision(digest)
    tests=read(OUT/'whole_hand_support_unit_verification.json')
    if tests['status']!='passed' or tests['core_source_sha256']!=sha(OUT/'whole_hand_support.py'):
        raise RuntimeError('Whole-hand core requires current successful analytical tests')
    collision=read(FOLDER.parent/'collision_convergence/manifest.json')
    if sha(OUT/collision['variant_model_path'])!=collision['variant_model_sha256']:
        raise RuntimeError('Separate model changed')
    bindings=read(ROOT/'team_integration_20261109/middleware_review/verification.json')
    if bindings['frozen_execution_bindings_verified']!=170 or bindings['frozen_execution_changes']:
        raise RuntimeError('Original execution sources require verification')
    pending=[]
    for task in TASKS:
        script=OUT/task['script'];output=OUT/task['output']
        entry=dict(task,script_sha256=sha(script),result_exists=output.exists())
        if output.exists():
            if sha(output.parent/task['snapshot'])!=sha(script):
                proof=revisions.get(task['script'])
                if not proof or sha(output.parent/task['snapshot'])!=proof['historical_source_sha256']:
                    raise RuntimeError('Existing result has another source version: '+task['name'])
                entry['historical_result_current_evaluator_revalidation']=proof
            if task['name']=='fresh_first_rotation_whole_hand_support' and read(output)['whole_hand_support_source_sha256']!=sha(OUT/'whole_hand_support.py'):
                raise RuntimeError('Existing native trial used another support core')
            if task['name']=='saved_state_whole_hand_contacts' and read(output)['support_source_sha256']!=sha(OUT/'whole_hand_support.py'):
                raise RuntimeError('Existing native state audit used another support core')
            if task['name']=='exact_recorded_ring_geometry' and read(output)['recorded_source_sha256']!=sha(ROOT/'ring_reference_readonly_20261109_124719_945552/result.json'):
                raise RuntimeError('Existing ring audit used another recorded reference')
        pending.append(entry)
    result=dict(status='offline_inputs_checked',native_dynamics_executed_by_this_check=False,
        analytical_tests=tests['tests_run'],tasks=pending,
        source_revision_evidence=revisions,
        exact_recorded_ring_source_sha256=sha(ROOT/'ring_reference_readonly_20261109_124719_945552/result.json'),
        no_WSL_launch_or_environment_selection=True,devices_opened=0,
        whole_cycle_ready_for_hardware=False,hardware_execution_allowed=False)
    (FOLDER/'queue_check.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--check',action='store_true')
    args=parser.parse_args();result=check()
    if args.check:
        print(json.dumps(dict(status=result['status'],analytical_tests=result['analytical_tests'],
                              pending_tasks=sum(not task['result_exists'] for task in result['tasks']),
                              native_dynamics_executed=False,hardware_execution_allowed=False)),flush=True)
        return
    if all(task['result_exists'] for task in result['tasks']):
        print(json.dumps(dict(status='existing_native_reports_verified',new_native_dynamics_executed=False,
                              hardware_execution_allowed=False)),flush=True)
        return
    try:
        import mujoco  # No environment fallback or dependency installation.
    except ImportError as error:
        raise SystemExit('Native MuJoCo environment required; no WSL launch, installation or device action was attempted') from error
    for task in result['tasks']:
        if task['result_exists']:continue
        old_argv=sys.argv
        try:
            sys.argv=[str(OUT/task['script'])]+task['args']
            runpy.run_path(str(OUT/task['script']),run_name='__main__')
        finally:sys.argv=old_argv
        if not (OUT/task['output']).exists():raise RuntimeError('Task did not produce its declared report')
    final=check();first=read(OUT/TASKS[-1]['output'])
    final.update(status='native_preflight_reports_created',native_dynamics_executed=True,
                 first_rotation_and_hold_passed=first['first_rotation_and_hold_passed'],
                 next_rotation_validated=False,whole_cycle_ready_for_hardware=False)
    (FOLDER/'native_execution.json').write_text(json.dumps(final,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(status=final['status'],first_rotation_and_hold_passed=final['first_rotation_and_hold_passed'],
                         hardware_execution_allowed=False)),flush=True)


if __name__=='__main__':main()
