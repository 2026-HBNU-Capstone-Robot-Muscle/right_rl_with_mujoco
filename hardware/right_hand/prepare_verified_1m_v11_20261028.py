"""Bind actual user baud change, independent rechecks and shorter-burst evidence."""
import hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
fresh=ROOT/'post_commission_readonly_20261028_041740_872756/result.json'
comparison=ROOT/'sync_burst_readonly_20261028_042609_020976/result.json'
old=ROOT/'link_commission_1m_20261028_041604_405637/result.json'
r=json.loads(old.read_bytes());f=json.loads(fresh.read_bytes());c=json.loads(comparison.read_bytes())
assert r['confirmed_ids']==list(range(1,17)) and f['status']=='all16_1m_off_reads_saved' and not f['differences']
assert c['blocks'][0]['complete']==1000 and not c['blocks'][0]['errors'] and c['blocks'][0]['group_size']==8
config=dict(status='all_16_verified',baud=1000000,baud_register_value=3,port='COM7',
    motor_ids=list(range(1,17)),torque_off_verified=True,
    evidence=fresh.relative_to(ROOT).as_posix(),evidence_sha256=hashlib.sha256(fresh.read_bytes()).hexdigest(),
    readonly_check=comparison.relative_to(ROOT).as_posix(),readonly_check_sha256=hashlib.sha256(comparison.read_bytes()).hexdigest(),
    original_user_commission=old.relative_to(ROOT).as_posix(),
    original_final_comparison_stopped=True,independent_two_reads_per_motor_all_preserved_values_match=True,
    full_burst_errors_retained='sync_burst_readonly_20261028_042302_708111/result.json',
    physical_or_host_root_cause_proven=False,motion_after_this_change_verified=False)
(ROOT/'hand_serial_link_20261028.json').write_bytes((json.dumps(config,indent=2)+'\n').encode())
gate=ROOT/'horizontal_quarter_gate_20261027.py';data=gate.read_bytes()
needle=b'read_recovery_episode_policy=dict(EPISODE_POLICY),'
assert data.count(needle)==2 and b'verified_serial_link_policy=link_review()' not in data
gate.write_bytes(data.replace(needle,needle+b'verified_serial_link_policy=link_review(),'))
from horizontal_quarter_plan_20261027 import OUT,HorizontalQuarterPlan
from horizontal_serial_link_20261028 import metadata
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from observed_book_model_policy import healthy_off
metadata()
for state in f['snapshots']:healthy_off(state)
OUT.mkdir(exist_ok=True)
request=dict(plan_version=HorizontalQuarterPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),
    origin='verified_user1Mbps_short_reply_bursts_after_two_stopped_v10_runs',
    source=fresh.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(fresh.read_bytes()).hexdigest(),
    off_state=f['snapshots'][-1],hardware_execution_allowed=False)
(OUT/'request.json').write_bytes((json.dumps(request,indent=2)+'\n').encode())
plan=HorizontalQuarterPlan(OUT/'request.json')
print(json.dumps(dict(status='v11_source_ready_model_pending',goals=len(plan.plan(plan.source_position())),
    host_baud=metadata()['baud'],motor_writes=0)))
