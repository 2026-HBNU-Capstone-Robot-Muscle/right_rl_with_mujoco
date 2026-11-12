"""Document the new physical power-wiring question, without device access."""
import datetime as dt,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261027/power_branch_20261027'

def main():
    folder=ROOT/'horizontal_quarter_trial_20261027_155220_214230';r=json.loads((folder/'result.json').read_bytes());last=r['samples'][-1]
    assert r['failed_phase']=='stage_3_empty_open' and r['all_torque_off_verified'] and r['settings_restored']
    OUT.mkdir(exist_ok=True)
    review=dict(status='awaiting_power_off_hub_photo_for_branch_wiring',timestamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat(),
        latest_user_trial=folder.name,error=r['error'],root_recovery_completed=True,stopped_phase=r['failed_phase'],cube_placed=False,
        minimum_voltage_v=min(v for s in r['samples'] for v in s['voltage_v']),final_voltage_v=last['voltage_v'],
        final_root_current_ma=[last['current_ma'][i] for i in (7,11,15)],final_root_tracking_rad=[last['tracking_rad'][i] for i in (7,11,15)],
        all_torque_off_verified=True,settings_restored=True,exact_voltage_loss_location_verified=False,
        user_proposed_counts=[3,13],interpretation='two motor chains fed separately from the same existing 5V TTL power hub',
        alternative_recommendation_counts=[8,8],alternative_reason='engineering preference for shorter, more balanced motor-chain power paths; improvement not guaranteed',
        wiring_changed=False,port_specific_instructions_ready=False,
        prerequisites=['5V motor supply OFF before unplugging or reconnecting motor cables','same existing 5V power source and common TTL 3-pin hub','U2D2 remains on the same TTL bus','identify actual hub ports and current chain boundary from a photo'],
        next_user_action='Power off and send a photo showing the blue hub, 3-pin connectors, and plugged-in cables.',
        next_motor_run_requested=False,after_wiring='PING/READ all16 with torque already OFF, then compare voltage before any new motion',
        official_sources=['https://emanual.robotis.com/docs/en/parts/interface/u2d2_power_hub/','https://emanual.robotis.com/docs/en/parts/interface/u2d2/','https://v1.leaphand.com/assembly'],
        hardware_execution_allowed=False,assistant_motor_writes=0,assistant_torque_or_setting_writes=0)
    (OUT/'review.json').write_text(json.dumps(review,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    guide='''# 같은5V 허브에서 두 갈래로 분배하는 개념

이번 v4는 뿌리 복귀223목표를 마치고 OPEN 초입12개 읽기 중 다시3.6V로 중단됐다. 마지막 뿌리 추종은 약0.064~0.071rad였고 각 전류는79/81/72mA다. OFF와 설정 복구는 확인됐다. 이 기록은 정확한 접점/전선 손실 위치를 증명하지 않는다.

사용자가 말한3개+13개가 모터 개수라면, 같은 기존5V 전원 허브의 공통 TTL3핀 단자에서 두 모터 묶음을 각각 연결하는 구성이 가능하다. 긴 직렬 전원 경로를 줄이려는 목적으로는 여건이 되면8개+8개(손가락 두 개씩)도 검토한다. 개수만으로 전압 개선을 보장하지 않는다. LEAP 공식 조립은 손가락별 중앙 케이블/전원 허브 연결을 소개한다.

```text
기존5V 전원 → 같은 파란 허브 ┬→ 모터3개 묶음
                            └→ 모터13개 묶음
U2D2의 TTL 통신도 같은 허브에 연결
```

이는 개념도이며 포트별 연결 지시는 아니다. 실제 포트와 현재 모터 연결 순서를 먼저 사진으로 확인한다. 전원 입력은 기존5V 한 개를 사용하고, 모터 케이블 변경은5V 공급을 끈 상태에서 진행한다. 모터ID/관절 매핑은 현재 값을 사용한다. 실제 배선 변경 뒤에는 모든16개의 응답과 토크OFF, 전압을 읽기만 한 뒤 다음 동작을 정한다. 지금 같은 회전 명령을 다시 요청하지 않는다.

공식 자료:
- https://emanual.robotis.com/docs/en/parts/interface/u2d2_power_hub/
- https://emanual.robotis.com/docs/en/parts/interface/u2d2/
- https://v1.leaphand.com/assembly
'''
    (OUT/'connection_concept.md').write_text(guide,encoding='utf-8')
    for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json','right_hand_rotation_20261026/fresh_empty_start_20261026_235258/progress_20261027.json'):
        path=ROOT/name;state=json.loads(path.read_bytes());state.update(status=review['status'],timestamp=review['timestamp'],latest_user_trial=folder.name,
            power_branch_review=(OUT/'review.json').relative_to(ROOT).as_posix(),next_user_action=review['next_user_action'],
            next_user_task=review['next_user_action'],next_user_command_ready=False,next_motion_command_ready=False,rotation_execution_ready=False,pending_user_input='power_off_hub_and_cables_photo')
        path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    block='''## CURRENT 2026-10-27: v4 뿌리 복귀 통과, 전원 두 갈래 연결 검토

- 최신 `horizontal_quarter_trial_20261027_155220_214230`은 뿌리 복귀223목표를 마쳤고 OPEN 초입에서3.6V 뒤 회복 실패로 멈췄다. 이전 검지 과도한 사전 이동 구간은 통과했다. 마지막 뿌리 전류79/81/72mA, 추종 약0.064~0.071rad이며 OFF/설정 복구가 확인됐다. 큐브 배치/회전 전이다. 추가적인 순서/한도 조정으로 같은 실행을 반복 요청하지 않는다.
- 사용자가3개+13개로 선을 나눌 의향을 밝혔다. 같은 기존5V 전원 허브에서 공통 TTL3핀 두 갈래로 각각 모터 묶음을 공급하는 의미라면 가능한 구성이다. 짧고 균형적인 경로를 위한8+8은 공학적 권고이며 개선을 보장하지 않는다. 실제 배선과 포트, 어느 그룹 경계인지 아직 확인되지 않았으므로 구체적인 모터/포트 이동 지시 전 전원을 끈 허브 사진이 다음 사용자 작업이다.
- ROBOTIS 공식 PHB 자료의 케이블 변경 전 전원OFF와 단일 전원 입력, U2D2의 동일 TTL 통신 채널, LEAP 공식 손가락별 허브 연결을 확인했다. 지금 모터/토크/설정 쓰기와 실제 배선 변경은0회다. 사진 확인 및 실제 변경 뒤에는16개 전체 PING/READ와 OFF 상태/전압을 먼저 확인한다. 현재 코드와 관절/건강 한도, 족자 문구는 변경하지 않았다.
- 근거: `right_hand_rotation_20261027/power_branch_20261027/review.json`, `connection_concept.md`. 기존 v4 모델 검토는 역사적 증거로 보존하며 실물 수평90도는 미검증, 수직 성공 총2회는 유지한다.

<!-- END POWER BRANCH REVIEW 20261027 -->

'''
    for name in ('professor_progress_20261015.md','sac_current_status.md'):
        path=ROOT/name;text=path.read_text(encoding='utf-8-sig')
        if '<!-- END POWER BRANCH REVIEW 20261027 -->' in text:raise ValueError('Already recorded')
        path.write_text(block+text.replace('## CURRENT 2026-10-27:','## PREVIOUS 2026-10-27:',1),encoding='utf-8')
    print(json.dumps(dict(status=review['status'],root_recovery_completed=True,minimum_voltage_v=review['minimum_voltage_v'],wiring_changed=False,assistant_motor_writes=0)))
if __name__=='__main__':main()
