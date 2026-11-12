"""Read saved evidence and write a compact, device-free progress summary."""
import json,hashlib,math
from pathlib import Path
from review_common_grip_selection_20261105 import PROTECTED
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261105/canonical_body_grip'

def span(values):return [min(values),max(values)] if values else None
def main():
    cases=[];missing=[];hashes=[]
    for front in (-15,-10,-5):
        p=O/f'full_placement/front{front}/review.json'
        if not p.exists():missing.append(str(p.relative_to(R)));continue
        report=json.loads(p.read_bytes());hashes.append(report['candidate_sha256'])
        if len(report['cases'])!=9:missing.append(f'front{front}: {len(report["cases"])}/9')
        for case in report['cases']:
            d=case.get('continuous');why=[]
            if not case['initial_placement']['passed']:why.append('initial_placement')
            if not d:why.append('continuous_replay_missing')
            else:
                if not d['safety_passed']:why.append('original_dynamic_safety')
                if d['regrasp_yaw_change_deg'] is None or abs(d['regrasp_yaw_change_deg'])>=5:why.append('regrasp_yaw')
                if d['final_up_tilt_deg']>=5:why.append('terminal_tilt')
                t=d['settled_tail']
                if t['minimum_upper_gap_sampled_75ms_mm']<=.3:why.append('sampled_layer_clearance')
                if t['yaw_drift_deg']>=3:why.append('settled_yaw_drift')
                if t['center_shift_mm']>=2:why.append('settled_translation')
                if t['opposed_lower_body_contact_fraction']<=.8:why.append('lower_body_contact')
                if t['separation_certificate_bad_steps_1ms']!=0:why.append('separating_plane_clearance_1ms')
            cases.append(dict(book_front_mm=front,gain_scale=case['gain_scale'],friction=case['friction'],passed=bool(d and d['whole_grip_task_passed'] and not why),reasons=why,continuous=d))
    ds=[c['continuous'] for c in cases if c['continuous']];centers=[d['final_center_world_m'] for d in ds];pair=max((math.dist(a,b)*1000 for a in centers for b in centers),default=0.);protected={p:hashlib.sha256((R/p).read_bytes()).hexdigest()==h for p,h in PROTECTED.items()};assert all(protected.values())
    summary=dict(status='offline_common_grip_continuous_evidence_summary',model_cases=len(cases),model_task_passed=sum(c['passed'] for c in cases),original_dynamic_safety_passed=sum(bool(c['continuous'] and c['continuous']['safety_passed']) for c in cases),missing=missing,all_candidate_hashes_identical=len(set(hashes))==1,candidate_sha256=hashes[0] if hashes else None,model_matrix_scope='3 book/support projections x3 gain scales x3 friction values; not 27 independent physical placements',regrasp_yaw_change_deg_range=span([d['regrasp_yaw_change_deg'] for d in ds if d['regrasp_yaw_change_deg'] is not None]),terminal_tilt_deg_range=span([d['final_up_tilt_deg'] for d in ds]),settled_tail_1ms_certificate_gap_mm_range=span([d['settled_tail']['minimum_separating_plane_gap_1ms_mm'] for d in ds]),maximum_terminal_center_spread_mm=pair,terminal_center_spread_is_model_sensitivity_not_physical_accuracy=True,failed_cases=[{k:v for k,v in c.items() if k!='continuous'} for c in cases if not c['passed']],physical_grip_verified=False,model_reorientation_nominally_about61deg_versus_physical_about90deg=True,upper_layer_load_is_hypothetical=True,placement_tolerance_not_yet_tested=True,protected_sources_unchanged=protected,hardware_execution_allowed=False,devices_opened=False)
    (O/'offline_summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n',encoding='utf-8');print(json.dumps(summary,ensure_ascii=False),flush=True)
if __name__=='__main__':main()
