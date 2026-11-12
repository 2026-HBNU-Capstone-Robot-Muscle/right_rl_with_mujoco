"""Construct an OFF-only model request for the user's newly changed posture."""
import hashlib,json
from pathlib import Path
from horizontal_quarter_plan_20261027 import HorizontalQuarterPlan,PREFIX_PHASES
from observed_book_model_policy import healthy_off
from hand_motor_mapping import SIM_TO_MOTOR_IDS
ROOT=Path(__file__).resolve().parent
source=ROOT/'repeat_off_readonly_20261027_204724_205068'/'result.json'
report=json.loads(source.read_text(encoding='utf-8'))
assert report['status']=='off_snapshots_saved' and report['servo_writes']==0 and report['maximum_drift_ticks']<=2
for state in report['snapshots']:healthy_off(state)
out=ROOT/'right_hand_rotation_20261027'/'edited_hand_204724';out.mkdir(exist_ok=True)
request=dict(plan_version=HorizontalQuarterPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),
    origin='user_reported_hand_reshaped_cube_removed_fresh_ping_read_only',
    source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    off_state=report['snapshots'][-1],hardware_execution_allowed=False)
path=out/'request.json';path.write_text(json.dumps(request,indent=2)+'\n',encoding='utf-8')
plan=HorizontalQuarterPlan(path);rows=plan.plan(plan.source_position())
phases={p:sum(1 for phase,_ in rows if phase==p) for p in dict(rows)}
print(json.dumps(dict(request=str(path),phase_counts=phases,assistant_motor_writes=0),ensure_ascii=False))
