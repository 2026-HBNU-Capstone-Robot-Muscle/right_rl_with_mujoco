"""Record the user's completed OFF-only restoration without more device work."""
import json,hashlib
from pathlib import Path
R=Path(__file__).resolve().parent
def main():
    p=R/'off_settings_restore_20261105_041900_498394/result.json';d=json.loads(p.read_bytes())
    assert d['status']=='original_off_RAM_settings_restored' and d['all_torque_off_verified'] and d['original_settings_verified']
    assert all(d[k]==0 for k in ('position_writes','torque_changes','mode_writes','EEPROM_writes'))
    assert hashlib.sha256((R/d['source']).read_bytes()).hexdigest()==d['source_sha256']
    followup=dict(status=d['status'],source=p.relative_to(R).as_posix(),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),setting_write_count=len(d['writes']),all_torque_off_verified=True,original_PID_restore_pending=False,assistant_motor_writes=0)
    out=R/'right_hand_rotation_20261105/preintegration/post_off_cleanup_fix/physical_restore_completion.json';out.write_text(json.dumps(followup,indent=2)+'\n',encoding='utf-8')
    paths=('right_hand_rotation_20261028/two_axis_connection/current_progress.json','right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json','right_hand_rotation_20261105/preintegration/progress.json')
    for rel in paths:
        path=R/rel;v=json.loads(path.read_bytes());v.update(status='original_settings_restored_two_forward_observations_body_grip_research_remaining',physical_observed_forward_hold_count=2,original_PID_restore_pending=False,latest_settings_restore=followup,next_user_action_required=False,next_user_command=None,next_user_command_ready=False,next_user_action='No immediate physical action needed; lower body grip and pitch/yaw connection remain offline work.');path.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    note='''## CURRENT 2026-11-05: 사용자 OFF 설정 복원 완료

- `off_settings_restore_20261105_041900_498394`에서 원래RAM설정 복원과16개모터 토크OFF를 확인했다. 목표각/토크/모드/EEPROM쓰기는0이며 이전P2100/D1900을 포함한 원래설정검증이 완료됐다. 이 명령의 추가실행이나 복원확인질문은 필요없다.
- 현재완료범위는 이번방향의 약4분의1회전과10초유지 관찰2회다. 윗층 회전공간을피하는몸통파지, 수직/수평경로의연결과재파지, 통합뒤왼손하중및양손간섭검증이남아있다. 지금즉시사용자물리작업은요청하지않는다.
- 보조자는장치를다시열거나모터쓰기명령을보내지않고 저장된복원결과만확인했다. 근거:`right_hand_rotation_20261105/preintegration/post_off_cleanup_fix/physical_restore_completion.json`.

'''
    path=R/'professor_progress_20261015.md';old=path.read_text(encoding='utf-8')
    if not old.startswith('## CURRENT 2026-11-05: 사용자 OFF 설정 복원 완료'):
        if old.startswith('## CURRENT'):old=old.replace('## CURRENT','## PREVIOUS',1)
        path.write_text(note+old,encoding='utf-8')
    print(json.dumps(followup,ensure_ascii=False))
if __name__=='__main__':main()
