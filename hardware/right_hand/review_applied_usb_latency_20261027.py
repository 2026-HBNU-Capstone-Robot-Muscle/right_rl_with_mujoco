"""Review saved latency change and unchanged operator model proof; no devices."""
import datetime as dt,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def main():
    current=json.loads((ROOT/'leap_usb_latency_result.json').read_text(encoding='utf-8'))
    if current.get('status')!='usb_latency_improved' or current.get('motor_writes')!=0 or current.get('motor_baud_changed') is not False:
        raise RuntimeError('USB change did not finish with verified unchanged motor settings')
    after=current.get('after_restart',current.get('after_reopen'))
    if not after or len(after.get('frames',[]))!=100 or not after.get('all16_baud_value5_3mbps') or after.get('communication_failures')!=0:
        raise RuntimeError('Missing100 clean full reads')
    from horizontal_quarter_plan_20261027 import HorizontalQuarterPlan,REQUEST,PROOF
    from horizontal_quarter_gate_20261027 import validate_review
    plan=HorizontalQuarterPlan(REQUEST);proof=validate_review(plan,PROOF)
    reference=plan.source_position().tolist()
    drift=max(abs(a-b) for row in after['frames'] for a,b in zip(row['position_ticks'],reference))
    report=dict(timestamp=dt.datetime.now().astimezone().isoformat(),status='usb_latency_applied_reading_faster_v5_proof_valid',
        user_apply_result=current['folder'],latency_timer_ms=current['latency_timer_ms'],
        median_before_ms=current['median_before_ms'],median_after_ms=current['median_after_ms'],
        read_time_reduction_percent=100*(1-current['median_after_ms']/current['median_before_ms']),
        p95_read_ms=current['p95_after_ms'],maximum_read_ms=current['maximum_after_ms'],
        read_frames=100,communication_failures=0,all16_baud_bps=3000000,
        minimum_idle_voltage_v=after['minimum_voltage_v'],maximum_position_delta_from_reviewed_source_ticks=drift,
        all_torque_off_verified=not any(after['after']['torque']),
        unchanged_operator_proof_valid=True,model_physical_rotation_verified=False,
        driver_restart_needed=current.get('exact_com7_restart_attempted',False),
        next_user_command='leap_horizontal_quarter_trial.cmd',
        next_task='EMPTY first SPACE; PLACE greenup/orangetowristID41 then SPACE; automaticOPEN then REMOVE/SPACE; send done only.',
        physical_horizontal_90_verified=False,physical_loaded_voltage_verified=False)
    folder=ROOT/'right_hand_rotation_20261027'/'usb_baud_latency_review'
    (folder/'applied_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False))
if __name__=='__main__':main()
