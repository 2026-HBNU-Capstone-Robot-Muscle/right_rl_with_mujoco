"""Compare the latest READ-only result with the reviewed start; no devices."""
import json
from pitch_handoff_plan_20261028 import ROOT,OUT,PitchHandoffPlan
from pitch_handoff_gate_20261028 import run_screen
from hand_motor_mapping import SIM_TO_MOTOR_IDS

def main():
    source=ROOT/'prefix638_off_readonly_20261028_215717_342391/result.json'
    r=json.loads(source.read_bytes());assert r['status']=='off_snapshots_saved'
    request=dict(plan_version=PitchHandoffPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=r['snapshots'][-1])
    path=OUT/'last_readonly_request.json';path.write_text(json.dumps(request,indent=2)+'\n',encoding='utf-8')
    p=PitchHandoffPlan(path)
    difference=(p.raw_off_ticks-p.source_position()).tolist()
    report=dict(source=source.parent.name,positions=p.raw_off_ticks.tolist(),review_center_reused=p.review_center_reused,
                differences_from_reviewed_center_ticks=difference,requires_new_model_center=not p.review_center_reused,
                devices_opened=False,motor_writes=0)
    if p.review_center_reused:
        proof=OUT/'last_readonly_cached_review.json'
        if not proof.exists():run_screen(path,proof,cancel=lambda:None)
    (OUT/'last_readonly_pose_comparison.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report))
if __name__=='__main__':main()
