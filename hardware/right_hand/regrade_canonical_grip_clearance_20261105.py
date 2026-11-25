"""Derived review based on the recorded exact-cylinder plane certificates.

Raw replay reports and all physics stay unchanged. A vertical-only lower bound
from the older clipped helper can be small even when radial separation proves
a larger genuine3D gap; it is not required in addition to a valid certificate.
"""
import argparse,json,hashlib,math
from pathlib import Path
from canonical_grip_task_review_20261105 import task_reasons
from review_common_grip_selection_20261105 import PROTECTED
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261105/canonical_body_grip'
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--tag',default='full_placement_v2');ap.add_argument('--output',default='certificate_review_v2.json');a=ap.parse_args();cases=[];raw=[];candidate_hashes=[]
    for front in (-15,-10,-5):
        p=O/a.tag/f'front{front}/review.json';report=json.loads(p.read_bytes());raw.append(dict(path=str(p.relative_to(R)),sha256=hashlib.sha256(p.read_bytes()).hexdigest()));candidate_hashes.append(report['candidate_sha256'])
        for c in report['cases']:
            d=c.get('continuous');why=task_reasons(d)
            if not c['initial_placement']['passed']:why.append('initial_placement')
            cases.append(dict(book_front_mm=front,gain_scale=c['gain_scale'],friction=c['friction'],legacy_task_passed=bool(d and d['whole_grip_task_passed']),certificate_based_task_passed=not why,reasons=why,continuous=d))
    ds=[c['continuous'] for c in cases if c['continuous']];centers=[d['final_center_world_m'] for d in ds];protected={p:hashlib.sha256((R/p).read_bytes()).hexdigest()==h for p,h in PROTECTED.items()};assert all(protected.values());assert len(set(candidate_hashes))==1
    result=dict(status='derived_exact_cylinder_certificate_review',source_matrix=a.tag,raw_reports=raw,raw_reports_not_modified=True,model_cases=len(cases),original_dynamic_safety_passed=sum(bool(c['continuous'] and c['continuous']['safety_passed']) for c in cases),legacy_task_passed=sum(c['legacy_task_passed'] for c in cases),certificate_based_task_passed=sum(c['certificate_based_task_passed'] for c in cases),candidate_sha256=candidate_hashes[0],reason_for_geometry_qualification='An exact-cylinder separating-plane certificate at every1ms state is sufficient. The old vertical-only gap can underestimate actual3D separation. No tolerance, collision mesh, mask, force, or native range is enlarged.',failed_cases=[{k:v for k,v in c.items() if k!='continuous'} for c in cases if not c['certificate_based_task_passed']],regrasp_yaw_range_deg=[min(d['regrasp_yaw_change_deg'] for d in ds),max(d['regrasp_yaw_change_deg'] for d in ds)],tilt_range_deg=[min(d['final_up_tilt_deg'] for d in ds),max(d['final_up_tilt_deg'] for d in ds)],certified_gap_lower_bound_range_mm=[min(d['settled_tail']['minimum_separating_plane_gap_1ms_mm'] for d in ds),max(d['settled_tail']['minimum_separating_plane_gap_1ms_mm'] for d in ds)],model_parameter_terminal_center_spread_mm=max(math.dist(x,y)*1000 for x in centers for y in centers),physical_grip_verified=False,physical90_or_actual_left_load_not_certified=True,protected_sources_unchanged=protected,hardware_execution_allowed=False,devices_opened=False)
    (O/a.output).write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in result.items() if k not in ('raw_reports','protected_sources_unchanged')},ensure_ascii=False),flush=True)
if __name__=='__main__':main()
