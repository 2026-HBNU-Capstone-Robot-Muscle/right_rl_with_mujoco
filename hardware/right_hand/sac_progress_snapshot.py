"""Summarize existing evidence only. Never imports hardware or marks readiness."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def main():
    hold=json.loads((ROOT/'id12_hold_analysis_result.json').read_text())
    probe=json.loads((ROOT/'sac_id12_replan_probe_result.json').read_text())
    refined=json.loads((ROOT/'sac_id12_replan_refined_result.json').read_text())
    before=hold['baseline']['error_end_rad'];after=hold['p600_trial']['error_end_rad']
    backoff_path=ROOT/'sac_id12_backoff_audit_result.json'
    backoff=json.loads(backoff_path.read_text()) if backoff_path.exists() else {}
    backoff_pass=backoff.get('all_passed') is True
    hardware_backoff_path=ROOT/'leap_id12_backoff_empty_result.json'
    hardware_backoff=json.loads(hardware_backoff_path.read_text()) if hardware_backoff_path.exists() else {}
    backoff_done=hardware_backoff.get('status')=='id12_backoff_complete'
    result=dict(
        scope='Latest semester-2: right hand supports cube and continuously reorients requested face up without reset; left hand turns upper layer; bimanual cube manipulation',
        whole_project_completion_percent=None,
        whole_project_estimate_note='Other team components have not been audited; no overall percentage established',
        completion_estimate_percent=30,
        estimate_kind='Subjective engineering estimate, not measured completion, success rate or time estimate',
        physical_sac_flip_verified=False,
        latest_id12_error_rad=after,
        fixed_goal_error_reduction_percent=100*(before-after)/before,
        nominal_search_candidates=len(probe['results']),
        nominal_search_passes=sum(bool(r['passed']) for r in probe['results']),
        broadly_validated_candidates=sum(bool(v['all_passed']) for v in probe['validation']),
        refined_difficult_case_passes=sum(bool(r['passed']) for r in refined['results']),
        backoff_model_audit_passed=backoff_pass,
        backoff_hardware_complete=backoff_done,
        next_step='Resolve possible physical contact and model geometry mismatch; obtain a focused thumb-middle view before new motion changes' if backoff_done else 'User runs one EMPTY ID12 return diagnostic via leap_id12_backoff_empty.cmd' if backoff_pass else 'Validate bounded return probe offline before any new hardware instruction',
        execution_blocker=None if backoff_pass else 'Backoff audit pending; prior approval review failed',
        hardware_run_requested=False if backoff_done else backoff_pass,
        motor_commands_sent=0)
    (ROOT/'sac_progress_snapshot.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
