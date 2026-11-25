"""Summarize completed one-factor probes, retaining every failed full replay."""
import hashlib,json
from pathlib import Path
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261105/canonical_body_grip';root=O/'continuous_sensitivity'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    bindings={};groups=[]
    for name in ('placement','friction','gain'):
        p=root/name/'review.json';d=json.loads(p.read_bytes());assert len(d['cases'])==d['planned_cases'];bindings[p.relative_to(R).as_posix()]=sha(p);rows=[]
        for i,c in enumerate(d['cases']):
            v=c['dynamic'];rows.append(dict(parameters=c['parameters'],passed=c['quarter_and_handoff_passed'],safety_passed=v['safety_passed'],completed_goals=v['completed_goals'],yaw_deg=v['final_yaw_from_placement_deg'],reasons=c['reasons'],guard_stop=v['stop'],worst=v['worst']))
            for f in ('initial_state.npz','end_state.npz'):
                p=root/name/str(i)/f;bindings[p.relative_to(R).as_posix()]=sha(p)
        groups.append(dict(group=name,cases=len(rows),passed=sum(c['passed'] for c in rows),original_dynamic_safety_passed=sum(c['safety_passed'] for c in rows),results=rows))
    nominal=json.loads((O/'continuous_handoff/review.json').read_bytes());report=dict(status='new_nominal_path_one_factor_sensitivity_complete',candidate_sha256=nominal['candidate_sha256'],groups=groups,total_additional_cases=sum(g['cases'] for g in groups),total_additional_passed=sum(g['passed'] for g in groups),nominal_baseline_passed=True,nominal_not_counted_as_an_additional_trial=True,one_factor_only_not_combined_parameter_matrix=True,model_hypotheses_not_calibrated_hardware_tolerances=True,physical_repeatability_not_verified=True,original90_reference_and_station_preserved=True,source_documents_sha256=bindings,devices_opened=False,hardware_execution_allowed=False);(root/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(groups=[{k:g[k] for k in ('group','cases','passed','original_dynamic_safety_passed')} for g in groups],total_additional=report['total_additional_cases'],passed=report['total_additional_passed'])),flush=True)
if __name__=='__main__':main()
