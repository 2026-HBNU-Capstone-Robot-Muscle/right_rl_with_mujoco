"""Update the ongoing status from saved evidence and the user's correction."""
import datetime as dt
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
folder=ROOT/'book_small_yaw_trial_20261024_215736_594865'
r=json.loads((folder/'actual_yaw_review.json').read_bytes())
entry=f'''## CURRENT {dt.datetime.now().strftime('%Y-%m-%d %H:%M')}: 小 회전 명령 완료 / 목표 방향 회전은 미확인

- `book_small_yaw_trial_20261024_215736_594865`에서 파지와60개 정지 목표,90개 접촉 보완 및 작은 회전 목표,40개 정지 목표를 모두 실행했다. 토크 OFF와 설정 복원은 확인했다. 영상1117프레임이 모두 유효하다. 최소 전압4.0V, 최대 전류104mA, 최대 추종0.084369rad다.
- 사용자는 마지막 SPACE 전에 큐브를 빼지 않았다고 명시했다. 따라서 원본의 `cube_removal_confirmed=true`는 SPACE 입력 기록일 뿐 실제 제거를 입증하지 않는다. 원본은 보존하고 `operator_correction.json`에 제거 절차 미완료 및 OFF 때 큐브가 남았음을 기록했다. OFF 뒤 움직임은 회전 성공 근거에서 제외했다. 이어 사용자가 별도 확인 질문에 "큐브 뺐어"라고 답했다.
- OFF 전 기준 정지와 회전 뒤 정지의 윗면 특징점 비교에서는 약2픽셀의 움직임이 나타났다. 고정 손바닥 특징의 이동은 약0.02픽셀이다. 큐브 특징점의 미세 이동은 있지만 정확한3차원 목표 방향이나3도 회전으로 판정하지 않는다. 윗면 색 스티커 격자는 빛 반사와 분리 불안정으로 전후 양쪽에서 일관된9점이 확보되지 않았다. 일부 기준 프레임에서8개 주변 타일 진단은 가능했으나 현재 ID40과 ID5 직접 동시 검출은 없어 모델 정합을 완료했다고 표기하지 않는다.
- 큐브를 뺀 후 `book_settled_off_readonly_20261024_220357_083205`에서 새 안정 빈손 OFF 각도를 직접 읽었다. 쓰기는0회이며 토크와 오류 및 watchdog은 모두0이다. 종료 직후 큐브가 남았던 수동 내려앉음 각도는 빈손 출발 승인으로 재사용하지 않는다.
- 같은 실제 파지 각도로 접촉 후보를 다시 계산했으며 일부 기존 가정에서는 세 손끝 접촉이 유지되지 않았다. 손끝 방향을 단순히 크게 늘리는 후보와 반대 방향 후보도 큐브 자체의 일관된 방향 조건을 만족하지 못해 실물 명령으로 내보내지 않았다. 실제 전류와 PID 및 전압과 추종 중단 기준은 변경하지 않았다. 현재 실제 큐브 움직임을 점수로 사용하는 제한 범위의 오프라인 동역학 경로 탐색을 진행 중이다. SAC 정책이 아니다.
- 비교 근거: `actual_yaw_review.json`, `operator_correction.json`, `before_OFF_comparison.jpg`, `paired_marker_diagnostic.json`. 첫 실행 `book_small_yaw_trial_20261024_215647_960137`는 검사 중 카메라 검은 프레임으로 중단됐고 모터 쓰기는0회였다.

'''.replace('小 회전','작은 회전')
for name in ('sac_current_status.md','professor_progress_20261015.md'):
    path=ROOT/name;prior=path.read_text(encoding='utf-8')
    if prior.startswith('## CURRENT'):prior=prior.replace('## CURRENT','## PREVIOUS',1)
    path.write_text(entry+prior,encoding='utf-8')
print('Saved actual trial review and user correction. Physical target yaw remains unverified.')
