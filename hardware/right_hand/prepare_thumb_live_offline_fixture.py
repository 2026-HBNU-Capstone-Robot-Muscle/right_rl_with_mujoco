"""Build an OFF-only simulation fixture; this does not authorize hardware."""
import json
from pathlib import Path
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from thumb_live_start_plan import REFERENCE,PLAN_VERSION

root=Path(__file__).resolve().parent
r=json.loads(REFERENCE.read_bytes())
out=root/'right_hand_preparation_20261021/fresh_start_fixture'
out.mkdir(exist_ok=True)
request=dict(plan_version=PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),
    origin='OFFLINE_FIXTURE_FROM_UNSETTLED_OFF_SAMPLE',
    off_state=r['final_off_snapshots'][-1],
    continuous_stationarity_verified=False,hardware_authorized=False)
(out/'request.json').write_text(json.dumps(request,indent=2),encoding='utf-8')
print(out/'request.json')
