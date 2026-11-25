"""Record a successful, strictly read-only OFF connection after operator setup."""
import json,datetime as dt,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'fast_off_readonly_20261029_222351_976318/result.json'
def main():
    r=json.loads(SOURCE.read_bytes());assert r['status']=='off_snapshots_saved' and len(r['snapshots'])==3
    assert not r['servo_writes'] and not r['torque_changes'] and r['maximum_drift_ticks']<=2
    assert all(all(int(v)==0 for v in row['torque']+row['error']+row['watchdog']) for row in r['snapshots'])
    nominal=json.loads((ROOT/'right_hand_rotation_20261029/gapped_inverse_cube/operator_request.json').read_bytes())['off_state']['position']
    delta=[int(a)-int(b) for a,b in zip(r['snapshots'][-1]['position'],nominal)]
    event=dict(timestamp=dt.datetime.now().astimezone().isoformat(),source=SOURCE.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        all16_torque_OFF=True,all16_error_and_watchdog_zero=True,stationary_three_snapshots=True,
        position_ticks=r['snapshots'][-1]['position'],delta_from_previous_prepared_start_ticks=delta,
        minimum_voltage_v=min(r['snapshots'][-1]['voltage'])*.1,maximum_voltage_v=max(r['snapshots'][-1]['voltage'])*.1,
        servo_writes=0,torque_changes=0)
    out=ROOT/'right_hand_rotation_20261029/contact_retained_inverse';(out/'connection_resumed.json').write_text(json.dumps(event,indent=2)+'\n',encoding='utf-8')
    for rel in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json','right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json','right_hand_rotation_20261029/contact_retained_inverse/research_progress.json'):
        path=ROOT/rel;p=json.loads(path.read_bytes());p.update(status='connected_OFF_inverse_contact_research_in_progress',current_status='connected_OFF_inverse_contact_research_in_progress',
            timestamp=event['timestamp'],next_command=None,next_user_command=None,next_user_command_ready=False,next_user_action_required=False,
            next_user_action='No physical action pending. Keep cube outside and fingers unchanged; model work is in progress.',
            current_motor_connection='All16 respond at1Mbps; all torque OFF/error0/watchdog0; three stable snapshots',
            responding_motor_ids=list(range(1,17)),connection_resumed_review=(out/'connection_resumed.json').relative_to(ROOT).as_posix(),
            ready_for_user_run=False,ready_for_supervised_operator_run=False)
        path.write_text(json.dumps(p,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    marker='<!-- inverse_connection_resumed_20261029_222351 -->\n'
    for name in ('professor_progress_20261015.md','sac_current_status.md'):
        path=ROOT/name;text=path.read_text(encoding='utf-8-sig')
        if not text.startswith(marker):path.write_text(marker+'사용자 전원 확인 뒤 모터16개 응답과 전부OFF/error0/watchdog0,3회자세안정을 읽기로 확인했다. 큐브 유지 실패 모델 조건을 보완 중이며 지금 추가 사용자 작업이나 실행 명령은 없다. 근거: right_hand_rotation_20261029/contact_retained_inverse/connection_resumed.json\n\n'+text,encoding='utf-8')
    print(json.dumps(event))
if __name__=='__main__':main()
