# 중지 기준 자세 캡처: 연결 오류 수정

- 최신 사용자 실행: `pose_pair_readonly_20261017_060405_039358/result.json`.
- 상태 stopped. 첫 snapshot motors=[]; 사진 없음. 새로운 기준 각도 미확보.
- 기록된 오류: `'NoneType' object has no attribute 'close'`.
- SDK `PortHandler.closePort()`는 `ser=None`이어도 `ser.close()` 호출.
  기존 finally가 원래 포트 개방 예외를 가릴 수 있었다. 당시 원래 예외는 유실됨.
- 후속 읽기 전용 포트 열거: COM3/COM4 Bluetooth만 존재, COM7 없음.
  포트 열거는 어떤 포트도 열거나 모터 통신을 하지 않음.
- 수정: 미개방 포트 종료 생략, 부분 개방 핸들 정리, 원래 예외 보존.
  카메라 시작 전 및 두 snapshot 직전 COM7 존재 확인. 자동 포트 변경 없음.
- 검증: test_readonly_port_failures, test_pose_pair_readonly,
  test_drive_readonly 총 12개 통과. 실제 하드웨어 재실행 없음.
- 다음 사용자 작업: USB 어댑터 재연결, 기존 기준 자세 유지,
  leap_middle_reference_readonly.cmd 실행 후 SPACE 한 번.
- ID12–엄지 끝모터, ID16–손바닥 접촉 확인은 유지한다.
  이번 실패로 새 영점/경로/게인 변경을 하지 않았다.
