"""Publish the tested local correction with its unresolved physical limits."""
import json
from nominal_cycle_plan_20261112 import ROOT,OUT,SOURCE,SOURCE_REVIEW,PROOF,NominalCyclePlan,sha
from nominal_cycle_gate_20261112 import validate_review

def put(path,value):path.write_text(json.dumps(value,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')

def main():
    proof=validate_review(NominalCyclePlan(OUT/'request.json'),PROOF)
    model=json.loads(SOURCE_REVIEW.read_bytes());tests=json.loads((SOURCE.parent/'software_verification.json').read_bytes())
    assert tests['passed'] and proof['accepted_for_operator_trial'] and not proof['tracking_bias_sensitivity_passed']
    (OUT/'software_verification.json').write_bytes((SOURCE.parent/'software_verification.json').read_bytes())
    ready=json.loads((OUT/'READY.json').read_bytes());ready.update(candidate_version='index8_support_v3',
        candidate_sha256=sha(SOURCE),physical_fall_fix_confirmed=False,tracking_bias_sensitivity_passed=False,
        limited_diagnostic_candidate_not_reliable_cycle=True,software_tests_passed=tests['passed'])
    put(OUT/'READY.json',ready)
    delivery=json.loads((OUT/'delivery.json').read_bytes());delivery.update(status='v3_nominal_trial_candidate_physical_fall_fix_unverified',
        candidate_version='index8_support_v3',candidate_sha256=sha(SOURCE),proof_sha256=sha(PROOF),
        physical_fall_fix_confirmed=False,tracking_bias_sensitivity_passed=False,assistant_started_motion=False)
    put(OUT/'delivery.json',delivery)
    first,last=model['holds']
    text=f'''# 연속 동작 시험 후보 v3

실행 파일: 루트의 `leap_cycle_nominal_two_turns.cmd`. 새 실행창 제목에 **검지 뿌리 지지 +최대8틱**이 표시된다. 보조자는 로봇이나 카메라를 열지 않았다.

낙하 직전의 검지 지지를 보정하려고 ID8 목표에 최대8틱(약0.70도)을 더했다. 앞1800지령은 그대로이고 이후200지령에 걸쳐 부드럽게 반영한다. v2와 비교해 다른15개 모터 목표, 시작 자세, 전체5089지령과 두 유지의 길이는 같다. 기존 ID16 최대2177틱도 유지한다. 이는 실제 낙하 원인을 확정하거나 낙하 해결을 보장한 수정이 아니다.

처음 배치부터 상태 초기화 없이381675개의1ms 상태를 독립 재검사했다. 명목 모델의 두 유지에서 각각4050/4050개 지지와 윗단 여유를 통과했다. 첫90도 목표 오차 {first['target_error_deg']:.3f}도, 최종180도 오차 {last['target_error_deg']:.3f}도다. 모델의 마지막 기울기는 {last['tilt_deg']:.3f}도이며 여유가 크지 않다.

**실물에서 기록된 추가 지연을 가정한 조건은 아직 미달이다.** 검지의 추가33틱 지연을 가정하면 첫 유지 목표 오차7.175도, 윗단 여유 인증3940/4050개로 실패했다. 이 가정은 모터 보정값이나 실물 큐브 상태 복원이 아니다. 손만 검사3조건도 변경 전과 같은 앞1728/1720/1718지령에서 실패한 기록을 보존했다. 해당 실패까지의 입력이 정확히 같다는 근거로 계승했으며 새 전체 손만 재생을 수행했다고 표시하지 않는다. 배치·마찰 오차 내성과 실물 두 회전 성공도 미검증이다.

엄지를 오래 고정하는 변경과 큰 검지 보정은 회전 부족 또는 낙하가 생겨 채택하지 않았다. 자세한 비교는 `../nominal_cycle_actual_review/support_transfer_fix_v3/`에 보존했다. 이전 v2의 실제 낙하와 이후 빈손 ID12 중단은 별개이며, 이번 수정으로 ID12 문제까지 해결됐다고 주장하지 않는다.

450mA, 기존PID, 최대2틱/75ms, 관절 범위와 전압·전류·온도·추종·통신 중단 기준은 그대로다. 최대500초 승인은 유지해 다시 입력할 필요 없다. 자동OPEN이나 자동 역회전은 없다. 저장 경로 재생이며 새SAC 학습·정책 전송·증류가 아니다.

## 기존 실행 순서

1. 큐브를 손 밖에 두고 `.cmd`를 새로 연다. 첫 SPACE로 현재 OFF 자세의 검사와 빈손 준비를 진행한다.
2. PLACE 안내 뒤 **초록면 위 / 주황면 손목ID41 방향**으로 놓고 손을 뗀 다음 두 번째 SPACE로 시작한다. 프로그램은 두 회전과 두 유지를 재생하며, 두 유지의 지령 길이는 각각4.05초다.
3. 큐브가 떨어지거나 이상하면 기존 Q/Esc/Ctrl+C로 중단한다. 마지막 안내 뒤에는 큐브 아래를 받치고 마지막 SPACE로 OFF·설정 복구 후 제거한다.

결과는 `nominal_two_turn_trial_날짜_시간/`에 저장된다. 프로그램 단계 진입을 실제 큐브 회전·지지 성공으로 세지 않는다. 손 위 지지에는 손바닥·뿌리·하우징 받침도 포함한다. 같은 범위의 한정 관찰용 후보이며 신뢰성 확보 완료나 전체 실물 테스트 완료가 아니다.

근거: `index8_support_v3/model_review.json`, `software_verification.json`, `source_preservation_check.json`, `review.json`. 이전 파일은 `before_index_support_fix_v3_20261112/`에 보존했다.
'''
    (OUT/'operator_guide.md').write_text(text,encoding='utf-8')
    summary=f'''# 낙하 구간 보정 v3

현재 실행 후보는 검지 뿌리 ID8 목표를 최대8틱(0.703125도) 보정한 `index8_support_v3`다. v2의 나머지15개 모터 목표,5089지령,2틱/75ms 및 모든 중단 기준을 유지했다. 실물에서 확인한 낙하 원인의 확정 해결이 아니라 작은 지지 보정의 시험 후보다.

독립 명목 재생: 두 회전·두 유지 통과,90도 오차{first['target_error_deg']:.3f}도/180도 오차{last['target_error_deg']:.3f}도, 마지막 기울기{last['tilt_deg']:.3f}도. 추가33틱 지연 가정: 첫 유지7.175도·윗단 인증3940/4050으로 미달. 실제 낙하 해결 및 ID12 중단 해결은 미확인이다. 소프트웨어7검사는 통과했고 원본170개와 직전 후보의변경대상외158개 결합 파일 해시를 확인했다.

채택하지 않은 비교: 엄지 지연80/160/240은 낙하 또는 약52~53도 회전 부족; 엄지 접촉강화1/5는 낙하 또는 약52.7도 부족; ID8+16은 첫 유지 약9.04도 부족, +32는 명목 낙하. 추가 지연 가정과 비교한 결과도 해당 폴더에 실패로 보존했다. +6은 별도 명목 모델 통과 기록이며 실제 개선을 검증한 결과가 아니다. 역방향 되짚기는 별도 계산을 종료했고 현재 같은 방향 연속 회전으로 바꾸지 않았다.

실행 안내: [operator_guide.md](../../nominal_cycle_trial_preparation/operator_guide.md). 보조자 장치 접근·모터쓰기·원격쓰기0. 기존실물stage1/stage2의관찰기록은보존한다.
'''
    research=ROOT/'right_hand_rotation_20261112/nominal_cycle_actual_review/support_transfer_fix_v3'
    (research/'REVIEW.md').write_text(summary,encoding='utf-8')
    print(json.dumps(dict(version='index8_support_v3',nominal_model_passed=True,software_tests=tests['tests'],
        tracking_bias_sensitivity_passed=False,physical_fall_fix_confirmed=False,devices_opened=0)))

if __name__=='__main__':main()
