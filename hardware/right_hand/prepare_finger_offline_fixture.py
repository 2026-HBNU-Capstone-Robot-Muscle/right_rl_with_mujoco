"""Use a measured stable OFF endpoint for the NEXT empty finger-path screen."""
import json
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from thumb_live_start_plan import ROOT
from hand_preparation_plan import PreparationPlan

source=ROOT/'thumb_prepare_current_empty_20261022_002034_279500/result.json'
r=json.loads(source.read_bytes())
assert r['all_torque_off_verified'] and r['settings_restored'] and r['final_off_reference_valid']
out=ROOT/'right_hand_preparation_20261022/finger_fixture';out.mkdir(parents=True,exist_ok=True)
request=dict(plan_version=PreparationPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),
             origin='OFFLINE_FIXTURE_FROM_RECORDED_OFF_ENDPOINT',hardware_authorized=False,
             off_state=r['final_off_snapshots'][-1])
(out/'request.json').write_text(json.dumps(request,indent=2),encoding='utf-8')
print(out/'request.json')
