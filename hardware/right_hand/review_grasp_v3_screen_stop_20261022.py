"""Summarize the stored 02:53 screen failure; never opens devices."""
import hashlib
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
folder=ROOT/'cube_grasp_trial_20261022_025316_928334'
r=json.loads((folder/'result.json').read_bytes());p=json.loads((folder/'current_start_screen.json').read_bytes())
s=p['static_screens']
report=dict(source_sha256=hashlib.sha256((folder/'result.json').read_bytes()).hexdigest(),
    status=r['status'],error=r['error'],write_events=len(r['write_events']),
    all_torque_off_verified=r['all_torque_off_verified'],settings_restored=r['settings_restored'],
    first_position=r['off_start_observations'][0]['position'],
    last_position=r['off_start_observations'][-1]['position'],
    maximum_initial_contact_mm=max(max(x['initial_self_contacts_mm'].values(),default=0) for x in s),
    maximum_step_increase_mm=max(x['self_escape_max_step_increase_mm'] for x in s),
    latest_contact_clear_goal=max(x['self_escape_complete_at'] for x in s),
    max_unapproved_range_violation_rad=max(x['unapproved_native_range_violation_rad'] for x in s),
    max_support_increase_mm=max(x['max_support_increase_mm'] for x in s),
    dynamic=p['dynamic_sensitivity'],hardware_commands_sent_by_assistant=0)
(folder/'stop_review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(report,ensure_ascii=False,indent=2))
