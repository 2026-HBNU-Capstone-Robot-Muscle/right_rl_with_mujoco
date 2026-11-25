"""Compact diagnostics for the latest saved book preparation screen."""
import json
from pathlib import Path

root=Path(__file__).resolve().parent
folders=sorted(root.glob('book_prepare_empty_*/result.json'),key=lambda p:p.parent.name,reverse=True)
latest=folders[0];r=json.loads(latest.read_bytes());folder=latest.parent
proof=folder/'current_start_screen.json';report=dict(folder=folder.name,status=r['status'],error=r.get('error'),
    all_torque_off_verified=r.get('all_torque_off_verified'),settings_writes_started='initial' in r,
    motor_motion_started='enabled_at' in r)
if proof.is_file():
    s=json.loads(proof.read_bytes());bad=[]
    for row in s['static_screens']:
        reasons=[]
        for key,limit in (('max_self_overlap_mm',1e-6),('max_support_after_lift_mm',.15),
            ('support_escape_max_unapproved_overlap_mm',1e-6),('support_escape_max_step_increase_mm',1e-6),
            ('native_range_violation_rad',1e-6),('max_goal_step_ticks',2)):
            if row[key]>limit:reasons.append(key)
        if not row['support_escape_complete']:reasons.append('support_escape_incomplete')
        if reasons:bad.append(dict(reasons=reasons,**row))
    old=json.loads((root/'right_hand_rotation_20261024/book20_current_review/timeout_fix_fixture/request.json').read_bytes())
    current=json.loads((folder/'fresh_off_request.json').read_bytes())
    report.update(static_cases=len(s['static_screens']),dynamic_cases=len(s['dynamic_sensitivity']),
        elapsed_seconds=s.get('elapsed_seconds'),failing_static_cases=len(bad),
        first_failures=bad[:4],dynamic_max_tracking=max(v['max_tracking_rad'] for v in s['dynamic_sensitivity']),
        changed_start_ticks=[dict(id=i+1,old=a,new=b,difference=b-a) for i,(a,b) in
            enumerate(zip(old['off_state']['position'],current['off_state']['position'])) if a!=b])
out=root/'right_hand_rotation_20261024/book20_current_review/latest_screen_failure_review.json'
out.write_text(json.dumps(report,indent=2),encoding='utf-8');print(json.dumps(report,indent=2))
