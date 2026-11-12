"""Stored relaxed OFF sample for simulation only, never live-start evidence."""
import json
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from prepared_placement_plan import SOURCE,PlacementPlan
from thumb_live_start_plan import ROOT

r=json.loads(SOURCE.read_bytes())
assert r['all_torque_off_verified'] and r['settings_restored']
out=ROOT/'right_hand_preparation_20261022/placement_fixture';out.mkdir(parents=True,exist_ok=True)
request=dict(plan_version=PlacementPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),
    origin='OFFLINE_FIXTURE_FROM_RELAXING_OFF_SAMPLE',hardware_authorized=False,
    off_state=r['final_off_snapshots'][-1])
(out/'request.json').write_text(json.dumps(request,indent=2),encoding='utf-8')
print(out/'request.json')
