# 미들웨어 공통 통신 복구

`hardware/right_hand`에 아래3개 모듈을 추가했다. 기존267개 원본과 회전/G 후보·제한·75ms·최대2틱·450mA/PID를 변경하지 않는다.

- `grip_transport_recovery_20261125.py`: 전체16개 개별 READ 대체, 송신/Windows UART 관찰, 운동 재시도와 분리한 OFF 확인.
- `serial_error_observer_20261125.py`: Windows에서 기존 pyserial의 오류 플래그를 관찰하며 새 드라이버 호출/설정 변경을 하지 않는다. 다른 OS에서는 관찰 기능을 사용하지 않는다.
- `grip_pause_resume_20261125.py`: 기존200ms 복구 뒤 최대1초 READ 대기. 새 전체16개 정상 상태2회와 목표/자세 검사 후 다음 미전송 목표부터 재개한다. 대기 중 모터 쓰기, 자동ON, 워치독 해제, USB 재연결은 없다.

`middleware-bridge`의 `RecordedAssets`는 세 모듈의 해시를 별도로 검사하고, `SourceRuntime`의 한 공통 버스와 검사기에 연결한다. G 준비/유지, 가로, 세로, 목표 면 선택의 복합 경로가 모두 이 연결을 사용한다. 따라서 미들웨어 쪽 변경과 이 세 파일을 함께 준비해야 한다. [미들웨어 안내](https://github.com/2026-HBNU-Capstone-Robot-Muscle/middleware-bridge/blob/feature/hardware/docs/recorded-motion.md)

기존 단독 `.cmd` 파일은 이번 변경으로 복구판으로 바뀌지 않는다. 이 변경의 적용 대상은 미들웨어 `recorded` 모드의 공통 제어 루프다. 단독 실행의 새 대기판과45초 읽기 진단은 원래 PC 작업 폴더에 별도로 보존되어 있다.

토크OFF 읽기 전용 실험에서는 약366ms/356ms 수신 끊김 두 구간을 복구해45초를 완료했다. 끊김의 물리 원인과 실제 토크ON/ROS 환경에서의 복구는 미확정이다. 미들웨어 소프트웨어 검사는 실제 제어 루프의 목표 배열 보존·복구 중 쓰기0·G 완료 조건·150ms 신선도와 복구 대기 경계·취소/정지를 확인한다. 이 문서는 실제 ROS 구동 성공을 주장하지 않는다.
