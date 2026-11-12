"""Publish reviewed next operator action; never opens a device."""
import datetime as dt,hashlib,json,sys
import numpy as np
from new_pair_grasp_plan_20261027 import ROOT,OUT,REQUEST,PROOF,NewPairGraspPlan,GRASP,REOPEN,HOLD
from new_pair_grasp_gate_20261027 import validate_proof
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def main():
    plan=NewPairGraspPlan(REQUEST);review=validate_proof(plan,PROOF);probe=OUT/'cache_probe';r2=validate_proof(NewPairGraspPlan(probe/'request.json'),probe/'review.json');assert r2['loaded_review_reused'] and r2['elapsed_seconds']<220
    protected={
      ROOT/'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
      ROOT/'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
      ROOT/'exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0',
      Path('C:/Users/김준서/Documents/GitHub/LEAP_Hand_API/python/sequence_flip.py'):'008f5a6b09b9f3231af6bff1c21a754ec8463b96a7efaa521dffd6bc88081656',
    }
    kept={str(path):dict(expected=value,actual=sha(path)) for path,value in protected.items()};assert all(v['expected']==v['actual'] for v in kept.values())
    rows=plan.plan(plan.source_position());command=f'& "{ROOT/"leap_new_pair_grasp.cmd"}"';nominal=next(v for v in review['loaded_physics'] if (v['book_front_mm'],v['gain_scale'],v['friction'])==(-10,1,.3));stamp=dt.datetime.now().astimezone().isoformat()
    ready=dict(timestamp=stamp,status='new_pair_grasp_ready_for_user_execution',command=command,plan_version=plan.PLAN_VERSION,source_request=str(REQUEST.relative_to(ROOT)),source_sha256=plan.SOURCE_SHA,review=str(PROOF.relative_to(ROOT)),review_sha256=sha(PROOF),model_cases=dict(static=201,hand_response=9,loaded=27),no_device_tests_passed=22,changed_start_cache_probe=dict(path=str((probe/'review.json').relative_to(ROOT)),fresh_static_cases=201,fresh_hand_response_cases=9,reused_loaded_cases=27,elapsed_seconds=r2['elapsed_seconds'],source_is_synthetic=True),goal_count=len(rows),nominal_motion_seconds=len(rows)*plan.SAMPLE_PERIOD,close_seconds=sum(p==GRASP for p,q in rows)*plan.SAMPLE_PERIOD,open_seconds=sum(p==REOPEN for p,q in rows)*plan.SAMPLE_PERIOD,grasp_hold_seconds=sum(p==HOLD for p,q in rows)*plan.SAMPLE_PERIOD,model_nominal_two_tip_contact_seconds=nominal['two_tip_hold_seconds'],physical_grasp_verified=False,physical_rotation_verified=False,physical_forward_flip_successes_retained=2,physical_horizontal_90_verified=False,cube_pose_friction_and_motor_model_unregistered=True,live_sac=False,assistant_motor_targets_torque_or_setting_writes=0,readonly_source='right_hand_rotation_20261027/grasp_off_035418/result.json',preserved_files=kept,guide='right_hand_rotation_20261027/new_pair_grasp_v1/guide/placement_guide.png')
    if '--publish' not in sys.argv:print(json.dumps(ready,ensure_ascii=False,indent=2));return
    marker='<!-- END NEW PAIR GRASP READY 20261027 -->'
    block=f'''## CURRENT 2026-10-27: 새 준비 자세의 큐브 파지 후 자동 열기 실행 준비 완료

- 방금 완료한 빈손 실물 실행은 `fresh_empty_grasp_20261027_032949_661470`이다. 최소 4.0V, 최대 111mA, 추종 0.09511rad, 토크 OFF/설정 복구 확인 상태를 유지한다. 같은 빈손 실행은 반복하지 않는다.
- 이후 PING/READ로만 현재 OFF 자세를 세 번 읽었다. 각도 차이는 최대 1틱이며 전부 토크 OFF/오류 0/워치독 0이다. 중력으로 내려온 ID4의 기존 수동 OFF 재진입 규칙을 사용해 안쪽으로 복귀하고 검지/중지/약지 뿌리를 받침에서 분리한다. 추가 관절 범위 확대나 전압/전류/추종 기준 완화는 없다.
- 다음 실행은 빈손 복귀/열기, PLACE와 두 번째 SPACE, 이미 빈손으로 도달한 동일 파지 끝 자세, 3초 관찰, 자동 열기, REMOVE와 세 번째 SPACE, OFF 순서다. 큐브는 초록 위/주황 손목 ID41 쪽으로 엄지와 중지 사이의 손바닥/뿌리에 자연스럽게 지지한다. 손으로 받쳐야만 머무르면 Q로 종료한다. 그림은 배치 가정이며 좌표 등록 완료가 아니다.
- 기하 201개, 손 응답 9개, 자유 큐브 접촉/열기 27개가 통과했다. 무한 강성으로 손을 고정하지 않고 원래 유한 강성 액추에이터와 접촉 힘을 유지했다. 단일 배치/55mm/100g 및 마찰 0.2/0.3/0.5, 구동 이득 0.5/1/1.5 가정이다. 명목 모델 두 손끝 접촉은 약 {nominal['two_tip_hold_seconds']:.2f}초였으나 실물 파지 확인은 아직 아니다.
- 장치 없는 실행 검사 22개가 통과했다. 기본 실행 무장치, 검토 없이 설정 쓰기 차단, 비상 OFF, 순서가 다른 목표/재전송 차단, 실측 엄지/뿌리 복귀 확인, PLACE/열기/REMOVE 순서, NaN/중복/누락/변조 검토 거부를 확인했다. 1틱 변화 합성 사례의 현재 복귀 210개 재검사도 통과했고 고정 열린 자세에서 시작하는 같은 파지/열기 27개만 재사용했다. 그 경로/코드/데이터가 다르면 재사용하지 않는다.
- 명목 동작은 {len(rows)*plan.SAMPLE_PERIOD:.1f}초이며 배치와 제거 대기는 별도다. 사용자에게 실제 실행 명령을 한 번 안내한다. 완료 후 `했어`만 받으면 `new_pair_grasp_.../result.json`과 영상을 검토한다. 새 추가 치수/촬영/로그 복사는 필요 없다.

```powershell
{command}
```

- `right_hand_rotation_20261027/new_pair_grasp_v1/review_v2.json`, `readiness.json`, `operator_guide.md`, `guide/placement_guide.png`에 근거와 안내를 저장했다. 수직 실물 성공 총 2회는 보존하고 수평90도는 미검증이다. 현재 제어는 검토한 경로 재생이며 실시간 SAC/카메라 피드백 학습이 아니다. 이번 턴의 모터 목표/토크/설정 쓰기는 0회다. 원본 시퀀스, 원본 구동 파일, 기본 관절 범위, 전시 족자 해시는 유지했다.

{marker}

'''
    for name in ('professor_progress_20261015.md','sac_current_status.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8-sig')
        if marker in old:raise RuntimeError('Readiness already published; refusing duplicate history')
        path.write_text(block+old.replace('## CURRENT 2026-10-27:','## PREVIOUS 2026-10-27:',1),encoding='utf-8')
    for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json','right_hand_rotation_20261026/fresh_empty_start_20261026_235258/progress_20261027.json'):
        path=ROOT/name;record=json.loads(path.read_bytes());record.update(timestamp=stamp,status=ready['status'],next_user_command=command,next_user_task='빈손으로 시작하고 PLACE 뒤에만 큐브 배치. 파지3초/자동열기 뒤 제거와 세 번째 SPACE. 완료 후 했어만 전송.',new_pair_grasp_operator_ready=True,new_pair_grasp_readiness=str((OUT/'readiness.json').relative_to(ROOT)),new_empty_grasp_operator_ready=False,new_empty_grasp_posture_physically_completed=True,physical_forward_flip_successes=2,horizontal_quarter_turn_verified=False,arbitrary_direction_rotation_verified=False,rotation_execution_ready=False,physical_new_pair_grasp_verified=False,live_sac_control=False,assistant_motor_writes=0);path.write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    (OUT/'readiness.json').write_text(json.dumps(ready,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(status=ready['status'],goal_count=len(rows),nominal_seconds=ready['nominal_motion_seconds'],cache_probe_seconds=r2['elapsed_seconds'],model_cases=ready['model_cases'],protected_files_unchanged=True,assistant_motor_writes=0),ensure_ascii=False,indent=2))
from pathlib import Path
if __name__=='__main__':main()
