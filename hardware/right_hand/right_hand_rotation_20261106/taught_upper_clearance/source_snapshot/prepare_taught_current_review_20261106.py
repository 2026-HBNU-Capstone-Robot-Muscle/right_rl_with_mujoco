"""Bind an empty-only offline recovery to the newly read, stationary hand."""
import json,hashlib
from pathlib import Path
import numpy as np
from observed_book_model_policy import healthy_off
from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK
from screen_calibrated_empty_margin_20261028 import OfflineRecovery
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261106/taught_upper_clearance';SOURCE=R/'resume_off_readonly_20261106_215211_977512/result.json'
def main():
    r=json.loads(SOURCE.read_bytes());assert r['status']=='readonly_data_saved' and r['servo_writes']==r['torque_changes']==r['maximum_drift_ticks']==0 and r['all_torque_off_read'];assert len(r['snapshots'])==3
    for s in r['snapshots']:healthy_off(s)
    out=O/'current_recovery_215211';out.mkdir(exist_ok=True);request=out/'request.json'
    obj=dict(plan_version=OfflineRecovery.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=r['snapshots'][-1],source=SOURCE.relative_to(R).as_posix(),source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),hardware_execution_allowed=False,cube_removed_confirmed_by_user=True)
    if request.exists():assert json.loads(request.read_bytes())==obj
    else:request.write_text(json.dumps(obj,indent=2)+'\n',encoding='utf-8')
    plan=OfflineRecovery(request);rows=plan.plan(plan.source_position());state=r['snapshots'][-1];baseline=np.load(O/'baseline_end.npz');canon=R/'right_hand_rotation_20261105/canonical_body_grip/continuous_handoff/end_state.npz';z=np.load(canon)
    report=dict(source=SOURCE.relative_to(R).as_posix(),all_16_responded=True,all_torque_off=True,position_drift_ticks=0,minimum_voltage_V=float(min(state['voltage']))/10,maximum_temperature_C=max(state['temperature']),position_ticks=state['position'],empty_recovery_goals=len(rows),nominal_empty_recovery_seconds=len(rows)*.075,empty_phases=list(dict.fromkeys(p for p,q in rows)),request=request.relative_to(R).as_posix(),current_natural_cube_xyz_m=baseline['qpos'][-7:-4].tolist(),prior_canonical_cube_xyz_m=z['qpos'][-7:-4].tolist(),prior_canonical_hand_q_sim=z['qpos'][:16].tolist(),devices_opened=False,hardware_execution_allowed=False)
    (out/'context.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report),flush=True)
if __name__=='__main__':main()
