"""Update durable local handoff from completed offline results only."""
from pathlib import Path
import json

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
MARK = '<!-- prehardware_cycle_20261109_latest -->'
END = '<!-- prehardware_cycle_20261109_latest_end -->'


def insert(path, text, after=None):
    source = path.read_text(encoding='utf-8')
    block = MARK + '\n' + text.rstrip() + '\n' + END + '\n\n'
    if MARK in source:
        begin = source.index(MARK)
        end = source.index(END, begin) + len(END)
        source = source[:begin] + source[end:].lstrip('\n')
    if after:
        point = source.index(after) + len(after)
        source = source[:point] + '\n\n' + block + source[point:].lstrip('\n')
    else:
        source = block + source
    path.write_text(source, encoding='utf-8')


def main():
    connected = json.loads((OUT / 'connected_sac_and_teacher/review.json').read_text())
    transition = json.loads((OUT / 'next_model_feedback_90_transition_regrasp/review.json').read_text())
    e = connected['endpoint']; h = connected['final_hold']
    if not connected['first_rotation_and_hold_passed']:
        raise RuntimeError('Do not publish a first-stage success if connected verification failed')
    if transition['success']:
        raise RuntimeError('A new success requires revising the interpretation before publishing')
    summary = f'''2026-11-09 최신: 실기 전 작업 요청으로 **현재2틱/75ms 조건의 새 SAC 학습을4회, 합계60,000환경 스텝 완료했다.** 이전의 “이번 새 학습 없음”은 아래에 남긴 과거 진단 단계의 기록이다. 이번 작업에서는 실물 모터·카메라를 열지 않았다.

**초기 배치부터 SAC6300목표와 별도 모델 교사109목표를 상태 초기화 없이 연결해 첫{e['yaw_deg']:.5f}도 회전→유지를 한 명목 모델 조건에서 통과했다.** 절대90도 목표 자세 오차{e['target90_error_deg']:.5f}도, 기울기{e['up_tilt_deg']:.5f}도, 공통 파지 위치 오차{connected['common_station_error_mm']:.5f}mm다. 마지막4.05초의4,050개1ms 상태에서 하부 손끝 접촉{h['lower']/h['samples']*100:.2f}%, 윗단 공간 분리 인증 하한 최소{h['gap']:.5f}mm였다. 끝109목표는 저장된 모델 교사 재생이며 전체를 순수 SAC 폐루프 또는 실물 성공으로 부르지 않는다.

SAC 단독 중간10256체크포인트는85.40도/자세 오차4.78도로 명목 조건만 통과했다. 초기X+0.5mm·마찰0.25·관측X+1mm 편향은 윗단 공간 미달이었다. 후속20k 강건화 학습과 다음5도용10k 두 실행은 전체 평가를 통과하지 못해 채택하지 않았다. 다음 절대180도 목표의90도 연결은 고정 접촉, 국소 모델 예측, 관절 역순 모두 미달이다. 이동 중 공간 점유/접촉 위치 변경을 허용한 추가 모델 예측도{transition['steps']}목표 후 목표 오차{transition['final']['error_deg']:.3f}도로 미달했다. 제한된 약지270개/중지270개 탐색의 후보0개를 모든 파지 불가능으로 해석하지 않는다.

저장 사진2,275장과 빈손550개 인코더 표본을 분석했고 실물 출력 없는 NumPy shadow 연결부14검사를 통과했다. 기존 카메라 내부 보정3개와 손 등록 자료를 읽었지만 손 등록은 미검증 상태이며 옛 코드의 ID41=30mm 등은 최신 약27mm와 다르다. 카메라 등록과 실제 힘 한도는 미검증이고450mA를 모델 토크로 환산하지 않았다. NumPy 이식은 증류가 아니며 **증류·다음 회전·실물 후보 준비 완료가 아니다.** 다음 작업은 접촉을 바꿔 다시 잡는 다음 회전 교사와 그 정책화/오차 검증이다. 지금 사용자 재연결·촬영·반복 실험을 요구하지 않는다.

상세: `right_hand_rotation_20261109/prehardware_policy_cycle/WORK_STATUS.md`, `review/index.html`, `evaluation_summary.json`. 원래170개 실행 결합 원본·실물450mA/PID/범위·중지 기준을 유지한다. 원격 저장소와 `apps/monitoring_ui`는 변경하지 않았다.'''
    insert(ROOT / 'cloud_handoff_20261106/CLOUD_START_PROMPT.md', summary, after='## 지금 먼저 할 일')
    insert(ROOT / 'professor_progress_20261015.md', '# 현재 작업: 실기 전 SAC 보정과 회전·유지 연결\n\n' + summary)
    insert(ROOT / 'right_hand_rotation_20261109/policy_audit/STATUS.md', '# 최신: 현재 조건의 SAC 학습과 첫 회전 연결\n\n' + summary)
    work = f'''# 실기 전 회전·유지·다음 회전: 2026-11-09

**첫90도 회전·유지의 명목 모델 연결은 통과했다. 다음90도와 편차 대응은 미완료다. 실물 시험 준비 완료로 판정하지 않는다.** 실물 모터·카메라·포트를 열지 않았고 원격 저장소와 미들웨어/UI를 변경하지 않았다.

## 실제 학습과 결과

| 실행 | 새 SAC 환경 스텝 | 평가 결과 |
|---|---:|---|
| `sac_current_contract_v4_20k` | 20,000 | 중간10256의 첫 회전·유지 명목 조건만 통과. 마지막20000은 윗단 공간 미달 |
| `sac_robust_current_20k` | 20,000 | 추가 배치·마찰·구동 편차와 공간 보상으로 학습했으나 두 체크포인트 모두 전체 경로에서 큐브80mm 이탈 |
| `sac_next_turn5_10k` | 10,000 | 다음5도 교육 단계의 최종 자세 오차4.49/4.87도, 미달 |
| `sac_next_fractional5_10k` | 10,000 | 틱 미만 행동 의도를 누적해도 최종 자세 오차8.84도 및 공간 미달 |

합계60,000스텝이다. 학습량은 성공 횟수나 완성도가 아니다. actor/정규화는 복원했고 critic/optimizer는 새로 시작했다. 앞선v1~v3는 학습기 호환성 오류로 갱신 전에 끝났으며 합산하지 않는다. 실패 기록과 학습 소스·체크포인트를 보존한다. 짧은 상태 창 평가를 초기 배치부터의 연속 성공으로 세지 않는다.

## 처음부터 연결한 첫 회전과 유지

`connected_sac_and_teacher/review.json`에서 처음6300목표는 매75ms 새 관측으로 SAC를 추론했다. 이전 평가와 모든 지령, 경계 qpos/qvel/ctrl/warmstart가 정확히 일치했다. 이어 별도 모델 예측 교사109목표를 재생했고 중간 큐브/손 상태를 초기화하거나 고정하지 않았다.

- 회전{e['yaw_deg']:.5f}도, 절대90도 전체 자세 오차{e['target90_error_deg']:.5f}도, 기울기{e['up_tilt_deg']:.5f}도.
- 공통 파지 위치 오차{connected['common_station_error_mm']:.5f}mm. 목표를 결과에 맞춰 옮기지 않았다.
- 마지막4.05초 하부 검지/엄지 손끝 접촉{h['lower']}/{h['samples']}, 약{h['lower']/h['samples']*100:.2f}%. 모델 법선력0.2N 기준이며 실물 지지 하중이 아니다.
- 같은4,050개1ms 상태에서 윗단 공간 분리 인증 통과. 보수적 인증 하한 최소{h['gap']:.5f}mm, 최종75ms 표본 기하 여유0.80586mm는 서로 다른 값이다. 실물 형상이나 연속 움직임을 인증한 것이 아니다.
- 유지 중 최대 방향 드리프트{h['yaw_drift']:.5f}도, 중심 드리프트{h['center_drift']:.5f}mm.
- 전 구간 자기/손–받침 겹침0, 최대 모델 큐브 침투0.68247mm, 최대 추종0.10077rad. 기존 native 관절·추종·접촉 기준을 통과했다.

총6409목표의 모델 제어 시간은480.675초다. 속도 최적화나 실물 성능 인증은 아니다. **끝109목표는 저장된 모델 교사 동작이므로 전체를 순수 SAC 폐루프 성공으로 부르지 않는다.** NumPy actor 이식은 증류가 아니다.

SAC 단독6300목표는85.4025도, 전체 자세 오차4.7752도, 마지막4.05초 하부 접촉92.52%였다. 공간 인증 재실행은 같은 명목 조건의 재현이며 독립 성공 조건을 더한 것이 아니다.

## 편차와 다음 회전

- SAC10256의 초기X+0.5mm, 마찰0.25, 관측X+1mm 편향은 모두 윗단 공간 미달이었다. 가정한 모델 조건이며 실물 허용 오차 측정이 아니다.
- 검지/엄지 접촉점을 고정한 다음±90도와, 첫 회전 후 관절 지령을 거꾸로 재생한929목표 비교는 통과하지 못했다. 역순 비교는 큐브80mm 이탈로 멈췄다.
- 제한된 약지270개와 중지270개 탐색에서 후보0개였다. 다른 손가락·큐브와 협력한 모든 파지가 불가능하다는 증거가 아니다. 사용자 수동 약지 값은 참고만 했고 ID14 범위를 확대하지 않았다.
- 상대5도 모델 교사는109목표/54개 유지 표본을 통과했다. 그 결과를 처음부터 이어 재검증한 것이 위6409목표다. 교사는 실제 동작보다 느린 모델 예측 제어이며 SAC가 아니다.
- 같은 방향의 다음90도, 절대180도 목표 모델 예측은126목표 후 오차85.27186도로 정체됐다.
- 이동 중 윗단 공간 점유와 접촉 위치 변경을 허용하고 최종 파지 기준을 유지한 비교도{transition['steps']}목표, 오차{transition['final']['error_deg']:.5f}도로 미달했다. 실제 형상/충돌과2틱/75ms를 완화하지 않았다. 이 결과로 모든 재파지의 불가능성을 주장하지 않는다.

## 저장 자료와 연결부

- 실물 실행2,891개 로그를 분류하고 빈손550개 표본의 추종 응답을 평가했다.12모터는 식별할 변화가 있었고4모터는 부족했다. 공유 시간표시/읽기 지연이 있으며 힘 한도·마찰·450mA 대응 토크는 식별하지 않았다.
- 사진2,275장을 읽어 마커를 검사했다. 배치 주변37프레임에서 손/큐브 마커가 함께 보였지만 파지 중 손 기준 마커는 없었다. 검증된 고정 등록의 재사용 가능성은 미확인이다. 픽셀을 근거 없이mm/각도로 바꾸거나 가림만으로 실시간 제어 불가능을 주장하지 않는다.
- 내부 보정3개 NPZ와 손 등록/세션 간 등록 자료를 읽었다. 손 등록은 `approximate_axis_correction_pending_visual_validation`, 자세 검증false, 오차 범위null이다. 옛 anchor 코드는 ID41=30mm/큐브54.5mm/큐브 마커10.6mm를 쓰며 최근 인계의 약27/55/10.7mm와 다르다. 옛 치수와 변환을 자동 채택하거나 카메라를 열지 않았다. `saved_calibration_asset_review.json`에 출처를 보존했다.
- 모터 출력 API가 없는 NumPy shadow 연결부14검사가 통과했다. 모터 순서, 학습 손바닥 기준점, 신선도,75ms/2틱, 확인 전 위상 유지, 실제 보낸 행동 이력, 파지 중 갱신을 검사했다.
- 5도 교육 평가의 초기각 라벨에 라디안을 도로 붙인 오류를 수정했다. 동일 초기 관측에서5.26837도로 계산하고 이전 값/수정 사유를 `curriculum_angle_unit_corrections.json`에 보존했다. 평가 성공 여부와 학습을 바꾸지 않았다.

## 남은 구체적인 작업

1. 첫90도 뒤 접촉을 바꿔 다시 잡고 절대 다음90도 및 공통 파지로 돌아오는 교사를 확보한다.
2. 좋은 교사를 정책에 학습시키고 처음부터 새 관측을 받는 전체 폐루프로 평가한다. 끝109목표의 저장 재생을 정책 완료로 표시하지 않는다.
3. 전체 연결의 배치·마찰·구동·관측 오차를 시험한다. 명목 모델만으로 실물 후보를 발급하지 않는다.
4. 기존 보정 자료 검토는 마쳤다. 미검증으로 남은 마커 장착 자세/정책 센서 변환과 최신 치수의 실물 관측 계약을 해결한다. 지금 사용자 재연결·재촬영은 요구하지 않는다.
5. 좋은 교사와 연결 정책을 확보한 뒤 증류와 별도 평가를 한다. 현재 증류 완료 기록은 없다.

[그림과 비교표](review/index.html) · [전체 결과](evaluation_summary.json) · [연속 검증](connected_sac_and_teacher/review.json) · [shadow 검사](shadow_runtime_verification.json).

실제450mA/PID/관절 범위/전압·전류·추종 중단 기준과170개 결합 원본은 유지한다. 모델의0.15/0.2196N·m를 실제 전류 환산값으로 쓰지 않는다. 향후 팀 하드웨어 작업은 `feature/hardware`에서 하며 `apps/monitoring_ui`는 건드리지 않는다.
'''
    (OUT / 'WORK_STATUS.md').write_text(work, encoding='utf-8')
    print(json.dumps({'handoff_updated':True,'work_status_updated':True,'devices_opened':0}), flush=True)


if __name__ == '__main__':
    main()
