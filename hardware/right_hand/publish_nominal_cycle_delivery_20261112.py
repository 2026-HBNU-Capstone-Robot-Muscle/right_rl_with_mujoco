"""Write the compact operator guide with actual proof limits, not a success claim."""
import json
from nominal_cycle_plan_20261112 import ROOT,OUT,SOURCE_REVIEW,PROOF,APPROVAL,sha
r=json.loads(PROOF.read_bytes()); nominal=json.loads(SOURCE_REVIEW.read_bytes()); proposal=json.loads((OUT/'proposal.json').read_bytes())
tests=json.loads((OUT/'software_verification.json').read_bytes())
if not tests['passed'] or json.loads(APPROVAL.read_bytes())['user_explicitly_approved']:
    raise RuntimeError('Expected tested, unapproved delivery')
lines=['# 전체 연속 동작의 명목조건 관찰 시험', '',
       '**실행 파일: 루트의 `leap_cycle_nominal_two_turns.cmd`.** 기존 stage1/stage2 파일은 바꾸지 않았다.', '',
       '이 파일은 처음 배치부터 회전→윗단 공간을 비운 유지→다음 회전→유지를 연결한 저장 모델5089목표를 재생한다. 이번에 실물에서 관찰한 stage2의 끝에 기존 다음 경로를 붙이는 방식과 다르다. 첫1417지령이 이전 실제 시험과 같다고 주장하지 않는다. 실물 성공·정밀90도·오차 내성은 아직 검증 전이다. 저장 경로 재생이며 새SAC 학습·실시간SAC·증류가 아니다.', '',
       '## 실행 전에 확인할 범위', '',
       f'- 보존된 명목 모델은381675개의1ms 상태에서 기존 관절·자기접촉·받침·추종·지지 기준을 검사했고 두 회전과 두 유지를 통과했다. 마지막180도 목표 오차는 {nominal["final180_error_deg"]:.3f}도, 다음 추가 수평 회전은 {nominal["additional_yaw_from_first_hold_deg"]:.3f}도다. 이는 실제 측정값이 아니다.',
       '- 작은 초기 배치·마찰 변화에서 실패했던 기록은 남아 있다. 모든 조건에 안전하거나 성공한다고 인증하지 않는다.',
       '- 큐브를 비활성화한 손 자체의 구동0.5/1/1.5 비교3개는 약0.00025~0.0016mm 모델 자기접촉으로 실패했다. 큐브가 있는 명목 모델의 통과와 구분한다. 이 실패를 통과로 바꾸거나 자기접촉 한계값을 확대하지 않았다.',
       f'- 빈손 준비 경로 정적 {len(r["empty_recovery"]["static_cases"])}조건·동적 {len(r["empty_recovery"]["dynamic_cases"])}조건과 소프트웨어 {tests["tests"]}검사는 통과했다. 현재 실제 시작각은 사용자가 실행할 때 다시 읽고 그 진입을 새로 검사한다.',
       f'- 저장 시작 기준 지령 시간은 {proposal["command_seconds_from_saved_seed"]:.3f}초(약7분27초)다. 기존300초 제한 안에 들어오지 않아 **이 시험에 한해 최대500초를 제안**한다.450mA/기존PID/2틱/75ms 및 전압·전류·온도·추종·통신 중단 기준은 동일하다. 모델0.15N·m를450mA의 환산으로 쓰지 않는다.', '',
       '## 순서', '',
       '1. 큐브를 손 밖에 두고 `.cmd`를 실행한다. 위 범위와500초 상한에 동의하는 경우에만 터미널에 **500 입력 후 Enter**. Enter만 누르면 장치를 열지 않고 취소한다. 보조자는 이 승인을 미리 입력하지 않았다.',
       '2. 기존과 같이 **첫 SPACE**로 빈손 시작 검사·준비. 실제OFF값에서 경로를 새로 검사한 뒤 동작한다.',
       '3. **PLACE가 뜬 뒤** 받침 자리에 초록면 위/주황면 손목ID41 방향으로 큐브를 놓고 손을 뗀 다음 **두 번째 SPACE**. 이 후 두 회전과 두 유지가 이어진다. 중간에 큐브를 다시 놓거나 관절을 손으로 옮기지 않는다. 두 유지의 지령 길이는 각각4.05초다.',
       '4. 마지막 관찰 종료 안내 뒤 큐브 아래를 받치고 **마지막 SPACE**로 OFF. 설정 복구와 종료 확인 뒤 큐브를 뺀다. 자동OPEN은 없다. 이상하면 기존 Q/Esc/Ctrl+C로 중단한다.', '',
       '처음500 입력은 기존 구동시간 제한 변경과 명목조건 한정 진단에 대한 사용자 동의다. 그 동의만으로 모터가 움직이지 않으며 실제 움직임은 이후 SPACE가 시작한다. 승인 전 기본검사·취소 동작에서는 모터와 카메라를 열지 않는 것을 소프트웨어에서 확인했다.', '',
       '결과는 `nominal_two_turn_trial_날짜_시간/result.json`에 저장된다. 첫 유지 `cube_first_cycle_hold.jpg`, 마지막 유지 `cube_second_cycle_hold.jpg`를 확인한다. 프로그램 완료를 목표 달성으로 자동 판정하지 않는다. 손 위 지지에는 손바닥·뿌리·하우징 받침도 포함한다. 실제 두 회전과 유지가 확인된 뒤 같은 조건의 반복·배치 차이를 별도로 평가한다.', '',
       '근거: `review.json`, `proposal.json`, `READY.json`, `software_verification.json`, `hand_gain*.json`. 명목 원본은 `initial_second_turn_hold_replay/review.json`이며 새 하드웨어 성공 기록이 아니다.']
(OUT/'operator_guide.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
(OUT/'delivery.json').write_text(json.dumps({'status':'operator_confirmed_nominal_diagnostic_prepared',
    'entry':'leap_cycle_nominal_two_turns.cmd','proof_sha256':sha(PROOF),'operator_approval_granted_by_assistant':False,
    'scope_confirmation_required_before_device_access':True,'original_frozen_files_modified':False,
    'physical_whole_cycle_success':False,'all_condition_safety_certified':False},indent=2)+'\n',encoding='utf-8')
print('Guide saved. Whole-cycle observation file prepared; operator scope confirmation still required.')
