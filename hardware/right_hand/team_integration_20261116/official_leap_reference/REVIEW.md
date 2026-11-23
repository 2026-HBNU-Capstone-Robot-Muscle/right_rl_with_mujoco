# 공식 LEAP 저장소 전반 검토 — 2026-11-16

사용자 요청에 따라 파지 작업은 중단하고 공식 API/Sim/Isaac Lab의 통신·좌표·정책 배포·학습 환경·ROS2 구조를 읽기 전용으로 검토했다. 실물 접근, 새 학습, 의존성 설치, 실행 코드나 모터 설정 변경은 하지 않았다. 조회 커밋과 읽은 소스는 manifest.json에 기록했다.

## 적용 가치

1. **좌표 변환을 독립적으로 검증한다.** API의 LEAPsim↔LEAPhand 변환은 ±π 영점 보정이다. Sim과 Lab 배포 코드에는 서로 다른 관절 순서 변환 배열이 있다. 공식 API ID는0~15, 현재 실물은1~16이며 현재 MuJoCo 순서 (8,6,7,5,12,10,11,9,16,14,15,13,4,3,2,1)와 명시적으로 대응해야 한다. 다른 엔진의 배열을 그대로 재사용하지 않는다. 현재 ticks-2048의 영점 원리는 공식 ±π 변환과 대응한다.
2. **학습과 배포의 입력·출력을 같은 정의로 묶는다.** Lab은 정규화 관절16+직전 목표16을3프레임 이어96차원으로 공급하며 처음 읽은 상태로 이전 목표/이력을 채운다. Sim 설정은102차원, phase와 history/targets 옵션이 있다. 배포는 직전 목표 + action_scale×action의 누적 상대 목표다. 현재 원본108/로컬110차원 정책과 직접 호환되는 체크포인트가 아니다. 현재 정책 파일의 관측 순서·이력 초기화·정규화·직전 목표와 실제 적용 명령의 일치 확인에 참고한다.
3. **실물 오차를 학습 환경에 넣는 방법을 참고한다.** Sim은 질량/질량중심/마찰/크기/PD를 무작위화한다. Lab은 성능에 따라 넓히는 ADR, 초기 위치/각도, 센서 잡음/바이어스, 외력, 동작 지연을 포함한다. Lab 설정에서 행동 지연 상한3주기, 관측 지연 범위0~0으로 기본 관측 지연은 비활성이다. 이 구조에 우리 실측 범위와75ms 제어를 넣는 것이 적용 방향이며 공식 범위가 현재 장치의 측정값은 아니다.
4. **통신 계층은 묶음 읽기와 데이터 신선도를 함께 본다.** API는 위치/속도/전류를 묶어 읽는 인터페이스를 제공한다. 다만 현재 공식 DynamixelReader.read는 실패 시 캐시된 이전 데이터를 반환하고, 일부 ID 데이터 누락 시 나머지만 갱신할 수 있다. 우리 완전16개 fresh 상태·오류 검사·중단 처리는 유지한다. API의 세 가지 값을 같이 읽는 구조는 미들웨어 데이터 공급에 유용하다.
5. **ROS2는 명령과 조회의 분리를 참고한다.** 공식 cmd_leap/cmd_allegro/cmd_ones 및 위치·속도·전류 조회 서비스를 통해 단위/좌표 의미가 드러난다. 우리 단일 시리얼 소유 어댑터에서 상태를 공유하는 구조에 참고한다. 공식 effort 응답은 read_cur의 전류이며 N·m 토크로 해석하지 않는다.
6. **기존 실물 PID와 공식 API의 공통점을 확인했다.** Python 예제 P600/I0/D200, 비엄지 기부3개0.75배와 현재 sequence_flip_checked 설정이 수치상 같다. 공식 Python/ROS2/Lab 예제 간 기본 P/전류 설정도 서로 다르다. 문서의 권장 수치 하나를 보편적인 정답으로 취급하지 않는다.

## 직접 이식 전 구분할 조건

- 공식 API/Python 및 Lab 배포 예제는4Mbps·ID0~15를 사용한다. 현재 장치는1Mbps·ID1~16이다. 현재450mA와 기존 PID는 변경하지 않았다.
- Sim 배포20Hz, Lab30Hz. 두 배포의 상대 action_scale은1/24rad로 최대 약27.16틱/추론이다. 현재2틱/75ms 경로와 시간·목표 분포가 다르므로 학습과 배포를 같은 간격/제약으로 설계해야 한다.
- Sim/Lab 제공 학습 설정은 PPO다. 우리 SAC 체크포인트를 공식 PPO 실행기에 직접 넣는 작업은 호환되지 않는다.
- Lab 문서의 실물 대상은7.5cm PLA 큐브281g, 우리 큐브는55mm다. 실제 질량은 이 검토에서 측정하지 않았다.
- 공식 Lab은 관절 고유감각만으로1축 연속 회전을 다룬다. 이 사례가 우리 정확한 목표각 정지·다음 방향 전환·양손 작업 완료를 보장하지 않는다. 논리적인 큐브 상태 갱신과 실제 자세/미끄럼 관측은 다른 정보다.
- 현재 MuJoCo 기반 검증을 버리거나 Isaac Lab 설치부터 시작할 근거는 이번 확인에서 얻지 않았다. 먼저 좌표 변환, 관측/목표 계약, 제어 간격 및 지연을 점검하는 데 적용 가치가 크다.

## 근거

- API: https://github.com/leap-hand/LEAP_Hand_API/blob/b0d00c881d0119077b2cab771a6de44f5aaec904/python/main.py
- 변환: https://github.com/leap-hand/LEAP_Hand_API/blob/b0d00c881d0119077b2cab771a6de44f5aaec904/python/leap_hand_utils/leap_hand_utils.py
- 읽기 실패 처리: https://github.com/leap-hand/LEAP_Hand_API/blob/b0d00c881d0119077b2cab771a6de44f5aaec904/python/leap_hand_utils/dynamixel_client.py#L385-L418
- Sim 배포: https://github.com/leap-hand/LEAP_Hand_Sim/blob/150bc3d4b61fd6619193ba5a8ef209f3609ced89/leapsim/deploy.py
- Sim 설정: https://github.com/leap-hand/LEAP_Hand_Sim/blob/150bc3d4b61fd6619193ba5a8ef209f3609ced89/leapsim/cfg/task/LeapHandRot.yaml
- Lab 배포: https://github.com/leap-hand/LEAP_Hand_Isaac_Lab/blob/c576e6a83183ba504aef8f57015f2899b799db1a/source/LEAP_Isaaclab/LEAP_Isaaclab/deployment_scripts/reorient_z.py
- Lab 환경/ADR: https://github.com/leap-hand/LEAP_Hand_Isaac_Lab/blob/c576e6a83183ba504aef8f57015f2899b799db1a/source/LEAP_Isaaclab/LEAP_Isaaclab/tasks/leap_hand_reorient/leap_hand_env_cfg.py

## 남은 사항

이 문서는 참고 검토이며 적용·실물 재시험 결과가 아니다. 현 장치의 전류 환산, USB 지연/반환 지연 설정, 정책 신선도와 전체 실물 제어 이력 일치는 아직 이번 비교로 검증하지 않았다.

