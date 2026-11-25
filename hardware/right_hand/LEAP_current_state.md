# LEAP 현재 상태 — 2026-10-05

## sequence_flip import 오류 수정

- 사용자가 `.runtime/python/python.exe sequence_flip.py` 실행 시
  `ModuleNotFoundError: main` 보고. 임베디드 Python의 python312._pth가
  스크립트 경로를 sys.path에 자동 추가하지 않는 것이 원인.
- 원본 GitHub 파일은 수정하지 않고 run_sequence_flip.py 및
  leap_sequence_flip.ps1/.cmd 추가. wrapper가 원본 python 폴더를
  sys.path 앞에 추가하고 main.LeapNode를 확인한 뒤 원본 스크립트를 실행.
  `run_sequence_flip.py --check`는 COM을 열지 않고 import 성공을 검증함.
- 시퀀스 실제 구동은 이 수정 이후 아직 하지 않음. 원본의3회 반복,
  PID/전류 덮어쓰기, 종료 후 torque ON 가능성은 그대로다.

---

## 시퀀스 중 카메라 시야 기록 준비

- 사용자 제안: 과거 실물에서 성공했던 sequence_flip.py 동작 중
  ID40+큐브2마커 연속 가시성을 카메라 단독 프로세스로 기록.
- 새 leap_sequence_vision.cmd/.ps1은 leap_vision_check.py의
  --sweep --sweep-seconds 90 --external-motion을 실행한다. 카메라 창
  SPACE로 시작,2초 구간별 진단과10초 구간별 원본 프레임을 저장.
  이 프로세스는 모터 COM을 열지 않는다. 기존 카메라 진단9검사 통과.
- 원본 sequence_flip.py는 별도 COM7 프로그램. 7자세,70보간×30ms,
  전체3회 반복. 원본 main.py 초기화가 P600/I0/D200,전류450mA로
  현재 SAC 세팅을 덮어쓰고 sequence_flip.py 종료 후 torque OFF 없음.
  영상검사와 동시실행은 가능하지만 이 점을 명시하고 SAC와 동시 실행하지 말 것.

---

## 21:07 단일 웹캠 실시간 시야 확인

- 사용자 요청으로 Streaming Webcam DSHOW/1280x720 MJPG를 모터 포트 없이 직접 열었다.
  첫 정지상태5초81프레임 중 ID40+큐브2마커66프레임(81.5%).
  이어20초 카메라 단독 캡처307프레임 중215프레임(70.0%); ID40은 전부 보임.
  큐브2마커 누락은 대개 ID1만 보이는 프레임. 저장한2초 간격 영상에서
  큐브가 실제 뒤집히는 변화가 보이지 않아 회전 경로 가림은 미검증.
  샘플: camera_live_20261005_210238.png, camera_sweep_20261005_210702/.
- 사용자가 준비한 뒤 카메라 창 SPACE로20초를 시작하는
  leap_vision_sweep.cmd/.ps1 추가. 모터 포트 없음. 손으로 큐브를 천천히
  예상 경로로 회전시키면2초 구간별 가시성/원본 이미지를 저장한다.
  영상 진단 관련9개 검사 및 Python 구문검사 통과. 실사용 결과는 미수집.

---

## SAC 상하·좌우 회전 가능성 점검

- 학습 관측은 시뮬의 큐브 위치/자세를 직접 읽고 카메라를 사용하지 않았다.
  원래 실물 deploy/vision/rubik_center/multi_camera.py는 가림을 줄이기 위해
  약90도 떨어진 복수 카메라를 지원하지만, 각 카메라가 자체적으로 손 ID40과
  큐브 마커2개를 읽은 결과 중 하나를 고른다. 1개+1개 융합은 구현돼 있지 않다.
- 현재 Windows leap_sac.cmd/CameraStream은 Streaming Webcam 한 대와
  camera1_720.npz(1280x720)만 사용한다. camera0.npz 및 camera1.npz는
  640x480 보정이므로 현재 720p 캡처에 직접 쓸 수 없다.
- 정책 관측에는 여섯 면 목표 quaternion이 정의돼 있으나 현재 Windows SAC
  실행은 FACE_GOAL_QUATS[0]만 넣고 결과에도 target_cube_face=0을 기록한다.
  즉 현재 파일은 연속적인 상하/좌우 목표 전환 기능이 아니다.
- 성공까지 남은 독립 과제는 (1) 실제 손가락 위/회전 경로에서 손 ID40과
  큐브2마커 연속 관측, 필요하면 두 번째 카메라 보정 및 Windows 통합,
  (2) 최근 ID12 추종 중단과 16개 watchdog 동시 만료 원인 점검,
  (3) ID0 한 면 목표의 실물 재배향 확인 후 여러 면 목표 전환 검증이다.
  마커 기준이나 추종 기준만 낮춰서 성공으로 간주하지 않는다.

---

## 20:40 SAC 구동 중단 — 배치/카메라 가시성 불일치

- 사용자 첨부 로그의 첫 시도는 준비 중 16개 watchdog raw255 만료로 중단, 정책0.
  두 번째 시도는 PLACE에서 READY를 간헐적으로 얻어 정책12회 전송 후
  ID12 추종 오차가 2초 안에 회복되지 않아 중단. 최종 토크OFF 16개와
  설정복원은 검증됨. ID12 원인은 아직 확정되지 않음.
- 두 번째 결과의 PLACE 영상 유효46/155프레임, 마커부족89프레임.
  정책 구간은 유효56/71프레임, 한 면만 검출된 프레임15개.
  사용자 관찰: 약45도 올라간 손가락의 검지/중지 첫마디 배치에서는
  현재 웹캠 시점으로 큐브 마커 한 면만 보이므로 실행 조건을 안정적으로
  만족시킬 수 없음. 과거 sequence_flip.py의 배치를 그대로 지시한 것은
  영상 관측 조건을 확인하지 않은 오류였음.
- 현재 배치 그대로 SAC 재실행하지 말 것. 큐브/손 지지와 ID40이 보이는
  상태에서 같은 Streaming Webcam의 위치/각도와 조명을 조정해 실제
  손가락 위 큐브 마커 두 면을 지속적으로 보이게 해야 함. 카메라 이동은
  ID40 기준 상대자세 계산에서 허용되지만 같은 720p MJPG/초점 유지.
  한 마커만 보일 때 정책 게이트를 해제하거나 큐브를 억지로 기울이지 말 것.
- 별도로 ID12 추종 중단 원인/물리 접촉 여부를 다음 구동 전 확인.
  SAC/PLACE 안내문은 배치를 관측 가능할 때만 사용하도록 수정.

---

## 20:37 idle watchdog 검사 완료 — SAC 차단 해제

- 사용자 leap_restore_watchdog.cmd 3회 실행. 마지막 결과 timestamp20:37:05,
  16개 preflight/postflight 모두 torque0/error0/watchdog0,4.8~5.0V.
  restore_ids=[]/write_events=[]: 복구 실행 전에 이미 전부0이었고
  이 세 번의 실행에서 모터 레지스터 쓰기 없음. 반복 실행 불필요.
- leap_sac.uncleared_off_failure() 현재 None. 다음 한 번 leap_sac.cmd 가능.
  큐브는 검지·중지 첫마디 위에 PLACE 후 사람 손 빼고 READY/SPACE.
  이전 전체 버스 통신 끊김 원인은 확정되지 않음; 재발 시 반복 시도하지 말고
  모터 전원 차단/저장 결과 분석.

---

## 최신 읽기 전용 결과: 16개 토크OFF, watchdog25 일부 잔존

- hand_readonly_check_result.json 확인: 16개 XC330-M288 모두 torque0,
  hardware_error0, voltage4.8~5.0V, 24~26C.
  watchdog raw0: IDs1~6,8. raw25: IDs7,9~16(9개).
  raw255 만료값은 없음. 첫 통신중단 후 watchdog 설정 복원이
  일부 모터에서 완료되지 않은 것으로 추정. 모터 고장 확정 아님.
- 결과 raw ID7=25라 이후 preflight의 ID7 register98 불일치를 설명한다.
  기존 read-only 체크 결과는 SAC latch를 풀지 않는다(raw0 조건).
- 새 leap_restore_watchdog.py/.ps1/.cmd: 전체16개 모델·torque0·error0·
  전압·온도를 먼저 확인. watchdog값0 또는25만 허용. 25인 ID만
  주소98에0을 단발 unicast write/즉시 readback. 관절목표/토크ON 쓰기 없음.
  종료 시16개 torque0/watchdog0 재확인; 실패면 STOP/전원 차단.
  새 검증 결과가 최신 OFF미확인 archive 이후면 SAC latch 해제.
  오프라인 관련16검사 통과. 실물 복원은 아직 미실행.
- 전원 완전 재인가 여부를 사용자에게 질문 중. 물리 전원/통신선 점검은 여전히 필요.

---

## 읽기 전용 검사 런처 수정

- 사용자는 hand_readonly_check.cmd 실행 시 Windows "현재 PC에서는 이 앱을 실행할 수 없습니다"
  화면을 봄. CMD가 PATH의 powershell.exe를 호출하는 구형 방식이었음.
- 정상 작동한 leap_sac.cmd와 같은 절대 PowerShell 경로로 변경,
  PS1은 현재 .runtime/python/python.exe -B 사용. 두 파일 존재/명령 일치/
  Python 모듈 import 확인. 모터 포트는 열지 않음.
- 기존 hand_readonly_check_result.json은 이번 시도에서 갱신되지 않았으므로
  OFF 실패 latch는 아직 유효. 수정된 hand_readonly_check.cmd 재실행 필요.

---

## 20:03~20:05 세 번 실행 — 모터 전원 재확인 전 SAC 실행 금지

- 사용자 첨부2a5202d9. 첫 실행은 준비 full_start_pose에서 SyncRead64
  통신패킷 오류 -3002(ID8 부근까지 일부 수신) 후 torque OFF readback이
  두 차례 -3001/ID1부터 무응답. OFF write는 전송됐지만 결과검증 실패.
  report leap_sac_20261005_200411_303170.json, all_torque_off_verified=false,
  정책명령0. 마지막 정상 sample은 모든 torque1/watchdog25/error0.
- 뒤 두 실행은 움직임 전에 ID7 레지스터98 예상0 불일치로 중지,
  정책명령0/설정쓰기0. 레지스터의 실제 원시값은 옛 로그에 저장되지 않음.
  두 실행의 final_torque_states에는 16개 모두0으로 저장돼 OFF는 검증됨.
  공식 XC330 문서: watchdog 만료는 raw255(-1), 0으로 해제 가능하지만
  실제값을 읽기 전에는 만료라고 확정하지 않는다.
- 사용자는 모터 전원을 물리적으로 끄도록 안내받음. OFF 여부 답변 대기.
  전원OFF→빈손·케이블 점검→전원 재인가 후 hand_readonly_check.cmd 실행.
  PING/READ만, 모터명령/리셋없음. 16개 torque0,hardware_error0,watchdog0
  모두 확인될 때만 SAC launcher latch 해제. 결과 hand_readonly_check_result.json.
- 기존 read-only 런처가 없는 Python 경로를 가리켜 .runtime/python/python.exe로 수정.
  watchdog98 raw를 16개 모두 출력/저장하도록 추가.
- leap_sac.py는 과거 unverified OFF archive가 최신 완전 read-only 결과보다
  새로우면 카메라/COM 접근 전에 중지. 뒤 두 실행의 전체OFF 검증은 사실이나
  ID7 watchdog 상태가 불일치하여 읽기 전용 상태점검까지 계속 차단.
  현재 데이터에서 직접 leap_sac.py 실행시 장치 접근 없이 exit1 확인.
  신규 latch 검사2개와 기존 SAC 검사10개 통과. 전체231검사 실행에서는
  기존 모의 launch 검사가 새 latch를 몰라1개 실패했고 fixture를 고쳐
  관련12개 검사를 재실행해 모두 통과. 전체 재실행은 불필요.

---

## 최신 19:34 실물 SAC 결과

- 사용자 새 정보: 예전 sequence_flip.py 성공 때 큐브를 2·3번째 손가락
  첫마디(검지·중지 근위지절) 위에 올려놓고 시작. 기존 손바닥 가운데 배치와 다름.
  leap_sac.py/loaded_policy_trial.py의 SAC PLACE 안내를 이 기준으로 수정.
  다음 배치가 정책 학습 분포와 완전히 일치한다고 주장하지 말 것.
- 학습 env/reorient.py reset은 큐브 nominal [0.1,0,0.05]m ±10mm,
  grasp_site는 XML [0.11,0,0.03]m. settle_steps=8 이후 실제 접촉 위치는
  이 정적 숫자만으로 확정 불가. 실물 hand/world 등록은 여전히 근사 후보.

- leap_sac_result.json: 20초 완료, 정책추론20/쓰기14, 영상63/129=48.84%,
  OFF/설정복원true. 단발 read timeout12회 모두 복구(ID16 11, ID14 1).
  읽기 반복 발생 원인은 아직 미확정. 전체 실행무중단이 곧 실전 안정성 증명은 아님.
- 사용자 관찰: 큐브는 손바닥에 그대로 있었음. 조작 실패.
  카메라 목표면 추정오차160.28→159.89도, 중심최대2.29mm.
  정책 목표는 시작 대비 최대416틱(36.56도)까지 변했으나 큐브 방향 거의 불변.
- 영상 실패: 마커부족42(ID4 누락26, ID1 누락16), 재투영24
  (모두3.00~3.21px). 프레임간격 중앙171ms, 검출 중앙78ms.
  학습20Hz 대비 실제129프레임/20초=6.4Hz, 유효추론20회=1Hz.
  손/큐브 등록은 여전히 근사 후보. 실측 palm-minus-cube 최초
  [16.5,21.7,7.6]mm. 학습 reset nominal grasp_site [110,0,30]mm,
  cube [100,0,50]mm±10mm 후 settle; 실측 y≈22mm는 시작 배치 차이 가능성.
- bounded_policy.py: 정책 쓰기가 없는 프레임의 중복 상태 읽기 제거.
  매 루프 첫 상태검사는 유지하고 실제 쓰기 직전 Control.apply 재검사는 유지.
  전체229개 오프라인 검사 통과(39.132초). 실시간 속도 개선은 아직 미검증.
  동일 배치로 반복구동해 성공을 주장하지 말 것.
- 다음 우선순위: 과거 성공 sequence_flip 시작 큐브 배치와 이번 배치 비교,
  파지 접촉/관측 등록/20Hz와 큰 차이의 제어 주기를 맞춘다.

---

목표: SAC 경로 정책으로 실물 큐브 목표 면 뒤집기. 사용자 이번주 완료 희망,
토큰 절감/짧은 답변/실행 명령 제공 선호. 완료 시점 보장 불가. 아래 최신 상태부터 읽을 것.

- 실행: outputs/leap_sac.cmd. 빈손SPACE→PLACE/배치→READY/SPACE→정책20초
  →REMOVE/제거/SPACE→OFF. 모터 실제 실행은 사용자 담당.
- Full XC330 ID1..16, COM7/3Mbps,550mA, PID 보존(P2100/I0/D1900).
  정책 학습 관절범위/명령당32틱/최소50ms. 임의 전류/관절 한계 확대 불필요.
- 정책: ../work/Leap_Test/policy/policy_np.npz = PPO 교사+residual SAC 증류망.
  온라인 학습/독립 SAC actor 아님. 목표ID0면 위. 초기 DEFAULT_POSE.
- 마지막 정책 구동14:52:20초 정상완료,8추론/7쓰기, 관절최대19.78도,
  영상40/138=28.99%, ID4누락97회. 큐브 재배향 성공 아님.
- raw_dense_threshold 검출 추가 후 저장33장 유효13→19/기존통과탈락0.
  실시간 효과 미확인. 마커2개/ID40/3px/200ms/이력3 검사는 그대로.
- 이후 두 실행 준비 중 SyncRead64 RX_TIMEOUT(-3001), 정책0,OFF/복원true.
  Full버스만 새 읽기1회 복구 추가: 첫 실패<100ms,전체<=200ms,
  10초 내 최대3회. 부분값폐기/새상태검사, 목표쓰기 및 torqueON 재전송 없음.
  read_failures에 진단. 통신이 계속 끊기면 정지. 실물 복구 결과 미확인.
- verify_leap_resume.py 전체229검사 OK(29.055초). 추가 모터 실행 없음.
- 다음 사용자 leap_sac.cmd 결과 확인: 복구 횟수/통신 실패ID, 유효영상율,
  실제 정책 빈도와 큐브 회전. 관측좌표/파지/제어주기 차이 해결을 우선.
  ±0.05/0.10 작은 범위 시험 반복은 목표가 아님.
- 학습 이력: ../work/Leap_Test/AI_session_Log.md(8월31일)에 증류망
  연속 목표달성5.98/10.89/13.52(0.11/0.16/0.21Nm) 기록.
  원래Linux학습PC ~/캡스톤_강화학습SAC, eval_final/*.json,
  videos_v57/distill/*.mp4. Windows에서는 전체 물리시뮬 재실행 안함.
- hand/world 등록 근사 후보 상태. 카메라 오차만으로 실물 성공 선언 금지.
- 사용자 성공 sequence_flip.py는 참고, 모터순서 변경없음. 후속자세 일부
  현재정책 관절한계 밖. 그냥 재생하거나 SAC라고 부르지 않는다.
- 작업 파일: hand_start_pose.py, hand_prepare_full.py, held_policy_shadow.py,
  bounded_policy.py, loaded_policy_trial.py, marker_detector.py, leap_sac.py.
- 셸: exec_command shell=cmd.exe/login=false. Python .runtime\python\python.exe -B.
  ../work 읽기만. 쓰기는 outputs. 전체 과거 로그는 LEAP_resume_status.md.
