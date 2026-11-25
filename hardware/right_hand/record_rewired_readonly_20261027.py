"""Record all16 PING/READ results after user-reported cable changes."""
import datetime as dt,hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def main():
    source=ROOT/'repeat_off_readonly_20261027_162601_077240/result.json';r=json.loads(source.read_bytes())
    before_source=ROOT/'horizontal_quarter_trial_20261027_155220_214230/result.json';before=json.loads(before_source.read_bytes())['initial']
    assert r['status']=='off_snapshots_saved' and len(r['snapshots'])==3 and r['servo_writes']==0 and r['torque_changes']==0
    snapshots=r['snapshots']
    assert all(len(s['position'])==16 and not any(s['torque']) and not any(s['error']) and not any(s['watchdog']) for s in snapshots)
    output=dict(status='all16_off_communication_and_idle_voltage_verified_after_user_rewiring',timestamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat(),
        source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        identified_motor_ids=list(range(1,17)),model_number=1240,fresh_snapshots=3,all_torque_off=True,all_hardware_errors_zero=True,all_watchdogs_zero=True,
        minimum_voltage_v=min(v/10 for s in snapshots for v in s['voltage']),maximum_voltage_v=max(v/10 for s in snapshots for v in s['voltage']),
        last_voltage_v=[v/10 for v in snapshots[-1]['voltage']],before_off_voltage_v=[v/10 for v in before['voltage']],
        before_off_minimum_voltage_v=min(before['voltage'])/10,before_off_maximum_voltage_v=max(before['voltage'])/10,
        maximum_drift_ticks=r['maximum_drift_ticks'],maximum_temperature_c=max(v for s in snapshots for v in s['temperature']),
        precise_read_times_ms=r['precise_read_times_ms'],user_also_reported_all_motors_in_dynamixel_wizard=True,
        actual_branch_topology_photo_verified=False,branch_counts_not_reported=True,under_load_power_verified=False,
        next_check='offline current-pose recovery, then user-started motion after Wizard releases COM7',
        assistant_motor_writes=0,assistant_torque_or_setting_writes=0,hardware_execution_allowed=False)
    out=ROOT/'right_hand_rotation_20261027/power_branch_20261027/after_rewire_readonly_review.json';out.write_text(json.dumps(output,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(status=output['status'],ids=16,voltage_v=[output['minimum_voltage_v'],output['maximum_voltage_v']],all_torque_off=True,under_load_verified=False,assistant_motor_writes=0)))
if __name__=='__main__':main()
