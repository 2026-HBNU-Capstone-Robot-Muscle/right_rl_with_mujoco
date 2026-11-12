"""Update the handoff from the saved exposure result; no devices."""
from pathlib import Path

ROOT=Path(__file__).resolve().parent
TITLE='## CURRENT 2026-10-23 16:18 이후: 노출 −7 해독 확인 / 사진·현재 각도 동시 저장 준비'
NOTE='''

- `camera_exposure_sweep_20261023_161815_427701` 정상 완료. 원래 노출 −5에서는 손바닥 ID40과 큐브 ID5가 함께 읽히지 않았고, −7에서는 6장 중 4장에서 둘 다 직접 해독됐다. 저장 JPEG 재검사에서도 4쌍을 확인했다. −8/−9 요청은 실제 읽은 값이 −7에 고정돼 지원 설정으로 취급하지 않았다. 원래 노출 −5·자동 getter −1 복원을 확인했다. getter −1의 자동/수동 의미는 미확인이다.
- 사진의 큐브는 손 위에 있다. 이 배치를 그대로 분석 자료로 보존하며 촬영 지시와 달랐다는 이유로 다시 파지시키지 않는다. 다만 이 카메라 전용 실행에는 현재 관절 각도가 없다. 몇 시간 전 04:46 관절 값을 16:18 사진에 붙이지 않는다. 직접 읽힌 두 마커의 평면 자세 후보는 모두 보존했고 아직 손 모델 좌표나 접촉을 확정하지 않았다.
- 새 `leap_cube_pose_readonly.cmd`는 **무동작 읽기 전용**이다. 지금 큐브/손을 그대로 두고 같은 구도의 손바닥 ID40과 초록 위 ID5를 보여준 뒤 터미널 SPACE 한 번으로 원본 사진 12장과 각 사진 직전/직후의 16개 관절을 저장한다. 사진 압축·마커 해독은 두 번의 읽기 이후에 한다. 토크는 이미 OFF여야 하며 USB/모터 전원은 읽기를 위해 연결한다. PING/READ만 사용하고 목표/토크/설정을 쓰지 않는다. ID41은 필요 없다.
- 노출 −7은 새 촬영의 별도 카메라 프로세스에만 적용하고 읽은 값을 확인하며, 종료 때 원래 설정 복원 결과를 저장한다. 기존 모터 실행본·공유 카메라 코드는 변경하지 않았다. 기존 04:46 실물 실행의 67정적/3응답 증명도 현재 코드와 일치한다. 시각은 드라이버 반환 시각이며 전후 3틱 이내만 정지 끝점 후보로 센다. 연속 정지·기하 보정·실물 회전·SAC 학습 성공으로 간주하지 않는다.
- 새 촬영과 기존 카메라/읽기 경로의 장치 없는 검사 **34개 통과**. SPACE가 프레임 사이에 눌려도 한 번만 누르면 되는지, 오래된 영상 배제, ON/오류 상태 중단, 각도 이동 제외, 정상/Q/읽기 오류/노출 변경 때 복원을 검사했다. 조수는 카메라·모터 포트를 열지 않았다.
- **지금 필요한 사용자 작업:** 다른 제어/카메라 프로그램 종료 → USB·모터 전원 연결(토크 OFF) → 현재 손/큐브 모양 유지 → 직전 사진처럼 ID40+ID5 함께 보이게 → `leap_cube_pose_readonly.cmd` → 터미널 SPACE 한 번 → 몇 초 정지. 종료 후 했어만 알려주면 저장 파일을 직접 읽는다. 미해독이어도 원본을 남기므로 바로 반복하지 않는다. 다음 계산은 이 동시 저장 자료로 모델과 큐브 접촉 위치를 좁히는 것이다.
- 근거: `right_hand_rotation_20261023/exposure_pose_capture_ready_review.json`, 노출 실행 폴더의 `exposure_review.json`, `leap_cube_pose_readonly_README.md`. 원하는 방향의 실물 큐브 회전은 여전히 미확인이고 현재 로봇 경로는 고정된 오프라인 경로다.

'''


def main():
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8')
        if old.startswith(TITLE):continue
        if old.startswith('## CURRENT '):old=old.replace('## CURRENT ','## PREVIOUS ',1)
        path.write_text(TITLE+NOTE+old,encoding='utf-8')
        print('Updated',name)


if __name__=='__main__':main()
