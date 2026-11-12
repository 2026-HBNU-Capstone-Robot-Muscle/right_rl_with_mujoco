"""Publish the verified PLACE-timeout recovery fix, with no device access."""
import datetime as dt,hashlib,json
from pathlib import Path
import numpy as np
from new_pair_grasp_plan_20261027 import ROOT,OUT,REQUEST,PROOF,OPEN_PATH,NewPairGraspPlan
from new_pair_grasp_gate_20261027 import validate_proof
def main():
    plan=NewPairGraspPlan(REQUEST);review=validate_proof(plan,PROOF);rows=plan.plan(plan.source_position());previous=json.loads((ROOT/'right_hand_rotation_20261027/new_pair_grasp_v1/readiness.json').read_bytes());preserved={}
    for name,v in previous['preserved_files'].items():
        actual=hashlib.sha256(Path(name).read_bytes()).hexdigest();assert actual==v['expected'];preserved[name]=actual
    readonly=json.loads((ROOT/'hand_readonly_check_result.json').read_bytes());motors=readonly['motors'];assert len(motors)==16 and all(not v['torque_enabled'] and not v['hardware_error'] and not v['bus_watchdog_raw'] for v in motors);now_angles=np.array([v['position_ticks_signed'] for v in motors]);difference=now_angles-plan.source_position();assert max(abs(difference))<=4
    stamp=dt.datetime.now().astimezone().isoformat();command=f'& "{ROOT/"leap_new_pair_grasp.cmd"}"';ready=dict(timestamp=stamp,status='place_timeout_recovery_v2_ready',command=command,source_request=REQUEST.relative_to(ROOT).as_posix(),review=PROOF.relative_to(ROOT).as_posix(),source_sha256=plan.SOURCE_SHA,review_sha256=hashlib.sha256(PROOF.read_bytes()).hexdigest(),model_cases=dict(static=201,hand_response=9,loaded=27),no_device_tests_passed=25,placement_wait_seconds=300,placement_callback_budget_seconds=320,motion_budget_seconds=180,goal_step_ticks=2,sample_period_s=.075,goal_count=len(rows),nominal_motion_seconds=len(rows)*plan.SAMPLE_PERIOD,last_readonly_difference_ticks=difference.tolist(),last_readonly_timestamp=readonly.get('timestamp'),assistant_motor_writes=0,physical_new_pair_grasp_verified=False,physical_horizontal_90_verified=False,forward_flip_successes_retained=2,previous_attempts={'new_pair_grasp_20261027_044740_174508':'EMPTY opening completed; PLACE wait90s expired; OFF/settings restored','new_pair_grasp_20261027_045408_418203':'old recovery exceeded motion/travel budget; no writes; OFF verified'},preserved_files_sha256=preserved)
    (OUT/'readiness.json').write_text(json.dumps(ready,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    block=f'''## CURRENT 2026-10-27: 큐브 배치 대기 만료와 재시작 경로 수정 v2

- 04:47 실행은 준비와 열림까지 완료했고 `place_cube timed out; OFF`로 끝났다. 90초 배치 대기 만료가 원인이며 토크 OFF/설정 복구가 확인됐다. 큐브 파지 구간은 시작하지 않았다. 04:54 재시도는 달라진 OFF 자세에서 옛 중간 자세를 거치는 경로가 시간/이동 한도를 넘어 실제 쓰기 0회로 거부됐다. 사용자가 손을 예전 자세로 맞출 필요는 없다.
- 배치 대기만 300초로 늘리고 30초마다 남은 시간을 표시한다. 대기 중에도 같은 건강 읽기/고정 관절 검사/Q를 유지한다. 배치 후 사진/3초 카운트다운까지 포함한 대기 예산은 320초로 맞췄다. 실제 동작 예산 180초, 450mA/PID, 목표2틱, 전압/전류/추종 중단 기준과 관절 범위는 바꾸지 않았다.
- 현재 OFF 각도에서 엄지를 복귀하고 뿌리를 분리한 뒤 열린 자세로 직접 이어진다. 옛 파지/OFF 중간 자세로 다시 돌아가 손을 말았다가 펴는 경로는 보내지 않는다. 명목 {len(rows)*plan.SAMPLE_PERIOD:.2f}초, {len(rows)}목표다. 실제 각도 PING/READ에서도 시작 차이 최대 {max(abs(difference))}틱, 전부 토크 OFF/오류0/워치독0을 확인했다. 이번 턴의 모터 쓰기는 0회다.
- 현재 출발점의 기하201/손 응답9/큐브 파지와 자동열기27 검사가 통과했다. 장치 없는25개 검사도 통과했으며 90초 이후 SPACE 허용, 긴 대기 중 건강 오류 전파, 현재 자세의 동작 시간 예산을 추가로 확인했다. 새 실물 파지와 수평90도는 여전히 미검증이며 수직 성공 총2회는 유지한다.
- 다음 사용자 동작은 같은 명령 1회다. 헤더 `NEW PAIR GRASP v2`를 확인한다. 빈손 첫SPACE, PLACE 뒤 배치/손뺀뒤 두 번째SPACE, 자동열기/REMOVE 뒤 큐브제거와 세 번째SPACE다. 로그 대신 `했어`만 받으면 저장 결과를 읽는다.

```powershell
{command}
```

- 근거와 안내: `right_hand_rotation_20261027/new_pair_grasp_v2/readiness.json`, `review_v2.json`, `operator_guide.md`. 원본 시퀀스와 구동 파일, 기본 관절 범위, 전시 족자 SHA256은 유지했다.

<!-- END PAIR PLACE TIMEOUT FIX V2 20261027 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8-sig');marker='<!-- END PAIR PLACE TIMEOUT FIX V2 20261027 -->'
        if marker in old:raise RuntimeError('Already published; refusing duplicate history')
        path.write_text(block+old.replace('## CURRENT 2026-10-27:','## PREVIOUS 2026-10-27:',1),encoding='utf-8')
    for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json','right_hand_rotation_20261026/fresh_empty_start_20261026_235258/progress_20261027.json'):
        path=ROOT/name;r=json.loads(path.read_bytes());r.update(timestamp=stamp,status=ready['status'],next_user_command=command,next_user_task='같은 명령의 v2 헤더 확인. 빈손 첫SPACE, PLACE 뒤 배치/손뺀뒤 두 번째SPACE, 자동열기 뒤 제거/세 번째SPACE. 배치 대기5분.',new_pair_grasp_operator_ready=True,new_pair_grasp_readiness=(OUT/'readiness.json').relative_to(ROOT).as_posix(),new_pair_grasp_plan_version=plan.PLAN_VERSION,placement_wait_seconds=300,physical_new_pair_grasp_verified=False,rotation_execution_ready=False,assistant_motor_writes=0);path.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(status=ready['status'],nominal_seconds=ready['nominal_motion_seconds'],placement_wait_seconds=300,model_cases=ready['model_cases'],tests=25,readonly_max_drift=int(max(abs(difference))),protected_files_unchanged=True),ensure_ascii=False,indent=2))
if __name__=='__main__':main()
