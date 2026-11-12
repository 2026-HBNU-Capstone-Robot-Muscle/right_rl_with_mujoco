"""Clear the answered measurement request; keep motion readiness unchanged."""
import datetime as dt,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261026'
def main():
    stamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat();obs=json.loads((OUT/'marker_dimension_observations_20261026.json').read_bytes());reply=obs['observations'][-1]
    for path in [OUT/'horizontal_next_step_20261026.json',OUT/'current_camera_and_yaw_review_20261026.json',OUT/'red_green_reference_completed_review_20261026.json',ROOT/'right_hand_rotation_20261024/current_handoff_20261025.json',ROOT/'right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json',ROOT/'right_hand_rotation_20261024/loaded_quarter_trial_20261025/readiness_review.json']:
        r=json.loads(path.read_bytes());r.update(timestamp=stamp,status='red_marker_approximate_measurement_received_offline_replay_in_progress',physical_measurement_not_yet_received=False,red_marker_user_measurement=reply,red_marker_nominal_for_sensitivity_mm=10.6,photo_9p8_not_accepted_as_measured_size=True,next_user_action=None,next_user_command=None,next_user_command_ready=False,next_motion_command_ready=False,horizontal_loaded_execution_ready=False,next_action='Continue saved-image corner, housing geometry and original measured-motion analysis. No additional measurement or capture requested.');path.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    block='''## CURRENT 2026-10-26: 빨강 ID2 약 10.5~10.7mm 답변 수신, 저장 기록 재생 계속

- 사용자가 빨강 ID2도 약 10.5~10.7mm라고 답했다. 정확한 캘리퍼스 측정으로 격상하지 않고 이 범위와 감도 계산용 10.6mm를 기록했다. 앞서 사진 외곽으로 계산한 9.8mm는 실물 확정 치수로 채택하지 않는다. 동일 치수 질문과 추가 촬영 요청을 종료했으며, 현재 사용자 다음 명령은 없다.
- 기존 충돌 상자와 원래 시각 CAD의 볼록 껍질을 같은 자세에서 대조했다. CAD 껍질이 단순 상자보다 겹침을 줄인다는 근거는 나오지 않았다. 볼록 껍질은 실제 빈 공간을 채울 수 있으므로 이 결과만으로 실제 접촉을 확정하거나 충돌 허용치를 넓히지 않는다. 라이브 검출기, 실행 모델, 모터 설정과 성공한 수직 동작은 그대로다.
- 예전 10월 11일 05:13 및 05:15 완주 기록의 명령값과 실제 읽기값을 모두 확보했다. 원본 첫 자세의 목표값과 실측값은 다른 형태를 만들었고 일부 겹침이 줄었지만 실제 읽기값에도 모델 겹침이 남았다. 명령한 이상적 궤적만으로 과거 실물 움직임을 판정하지 않고 실제 관절 궤적과 당시 큐브 사진을 함께 재생하는 작업을 계속한다.
- 치수와 좌표는 여전히 근사이며 수평 90도 실행 후보는 아직 활성화하지 않았다. 실물 수직 뒤집기 성공 총 2회는 보존했다. 다음 사용자 작업이 필요해지기 전까지 추가 모터 실행이나 사진을 요구하지 않는다.

<!-- END RED SIZE REPLY 20261026 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8-sig');marker='<!-- END RED SIZE REPLY 20261026 -->'
        if marker in old:old=old.split(marker,1)[1].lstrip('\n')
        path.write_text(block+old.replace('## CURRENT ','## PREVIOUS ',1),encoding='utf-8')
    print('Red range recorded; measurement/capture request cleared; no motion command activated.')
if __name__=='__main__':main()
