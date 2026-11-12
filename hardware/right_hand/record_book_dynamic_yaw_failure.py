"""Preserve the task failure and the concrete read-only geometry check needed."""
import datetime as dt
import json
from pathlib import Path
root=Path(__file__).resolve().parent
folder=root/'book_dynamic_yaw_trial_20261024_222132_806567'
review=json.loads((folder/'actual_yaw_review.json').read_bytes())
contact=json.loads((folder/'housing_contact_phase_summary.json').read_bytes())
entry=f'''## CURRENT {dt.datetime.now().strftime('%Y-%m-%d %H:%M')}: v2 실물 목표 회전 실패 / 중지와 약지 외형 검사 누락 확인

- 사용자는 v2 실행 후 중지와 약지가 닿는 것 같다고 말하며 시뮬레이션에서는 되는지, 실제 회전은 안 되는 것인지 물었다. `book_dynamic_yaw_trial_20261024_222132_806567`은 명령 전체와 OFF 및 설정 복원을 완료했으나 **목표 회전 과제는 실패로 기록한다.** 영상1180프레임 모두 유효, 최소4.1V, 최대95mA, 최대 추종0.079767rad다. 중단 기준에 걸린 실행이 아니다.
- OFF 전8쌍의 기준 및 사후 윗면 특징점 비교에서 주로 약4.2픽셀 옆 이동이 나타났다. 영상 x방향 선의 각도 변화 중앙값은 약-0.09도지만 이것은3차원 중력축 yaw 측정값이 아니다. 완전한0도 정지까지 입증했다는 뜻은 아니며, 의도한 확실한 방향 회전 성공은 아니다. 전후 비교는 `before_OFF_comparison.jpg`, 수치는 `actual_yaw_review.json`이다.
- 기존 후보의 활성 큐브 시뮬레이션은 파지 기준 실제 각도에 고정 추종 오차를 적용한 손 궤적을 지정하고 큐브 접촉을 계산했다. 실제 모터 응답이나 마찰 및 배치를 맞춘 완전한 실물 재현이 아니다. 최신 실제 손 궤적을 그대로 넣은 진단에서도 모델은 약2도 회전을 예측해 실제 영상과 차이가 남는다. 이 차이를 모터 추종만의 문제로 단정하거나 시뮬레이션 통과를 실물 성공 근거로 쓰지 않는다.
- `book_support_model_20261024.py`의 추가 모터 외형 검사는 중지와 엄지에만 적용돼 있었다. 원래 충돌 기본 도형 검사는 그대로 있었지만, 중지와 약지 사이의 추가 시각 외형 검사가 빠져 있었다. 별도 장치 없는 진단에 중지와 약지 메시 외형만 추가하자 중지 말단 및 중간 링크와 약지 뿌리 외형의 볼록껍질 겹침이 나타났다. `housing_contact_phase_summary.json`은 파지 전후 및 회전 구간과 사람 제거 구간을 구분한다. 볼록껍질은 오목한 모터 구조를 과대평가할 수 있으므로 이 수치를 실제 침투 깊이나 실제 충돌 확정으로 부르지 않는다. 모델 간섭이 없다는 기존 판단의 한계가 확인된 것이다.
- **다음 사용자 작업:** 큐브는 뺀 상태, 토크는 현재 OFF 상태로 두고 손가락과 받침은 움직이지 않는다. 카메라만 옆으로 옮겨 중지와 약지의 뿌리부터 끝까지 및 손바닥 가장자리가 보이게 한다. `leap_middle_ring_side_readonly.cmd` 실행 후 SPACE 한 번이면 사진1초 묶음과 전후 관절 읽기를 저장한다. USB와5V전원은 읽기를 위해 연결하되 이 프로그램은 목표나 토크 및 설정 쓰기를 하지 않는다. 기존 시선에서 가려진 외형과 실제 관절 자세를 비교하기 위한 확인이며 회전 재시도가 아니다.
- 읽기 전용 사진 도구의 해당 뷰 옵션을 추가했고 기존 전후 읽기 및 포트 실패 검사14개를 통과했다. 기본 검사 실행은 장치를 열지 않았다. 이번 분석에서 실물 쓰기는0회다. 새 방향 명령을 더 크게 만들어 반복하지 않고 실제 외형과 접촉 가정부터 맞춘다. 전시 문구는 변경하지 않았다.
- 근거: `transfer_diagnostic.json`, `housing_contact_phase_summary.json`, `actual_yaw_review.json`, `leap_middle_ring_side_readonly.cmd`. 다음 회전 경로는 아직 준비 완료로 표시하지 않는다.

'''
for name in ('sac_current_status.md','professor_progress_20261015.md'):
    path=root/name;prior=path.read_text(encoding='utf-8')
    if prior.startswith('## CURRENT'):prior=prior.replace('## CURRENT','## PREVIOUS',1)
    path.write_text(entry+prior,encoding='utf-8')
print('Recorded physical rotation task failure and next read-only side-view action.')
