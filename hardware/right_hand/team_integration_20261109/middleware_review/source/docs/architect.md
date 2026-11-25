middleware-bridge/                      # ROS 2 통합 프로젝트
├── .gitattributes
├── .gitignore
├── README.md
├── pixi.toml                         # Windows (win-64)용 ROS 2 Humble 환경 및 작업 정의
├── pixi.lock                         # 고정된 Pixi 환경 의존성
├── .pixi/                            # 로컬 Pixi 환경 생성물 (Git 제외)
├── .colcon-win/                      # Windows colcon 빌드·설치·로그 생성물 (Git 제외)
├── apps/
│   └── monitoring_ui/
│       ├── index.html                # ROS 2 연결 점검 화면
│       ├── styles.css                # 반응형 어두운 테마 스타일
│       ├── README.md                 # Tauri 실행 방법 및 검증 범위
│       ├── scripts/build.ps1         # PowerShell 기반 Tauri 정적 파일 생성기
│       ├── src/main.js               # rosbridge 토픽·서비스 WebSocket 통신
│       └── src-tauri/
│           ├── Cargo.toml            # Tauri 2 Rust 의존성
│           ├── build.rs
│           ├── icons/icon.ico        # Windows 큐브 아이콘
│           ├── tauri.conf.json       # 창 설정 및 localhost WebSocket CSP
│           └── src/main.rs           # Tauri 애플리케이션 시작점
├── docs/
│   ├── architect.md                  # 프로젝트 트리 및 아키텍처 현황
│   ├── request.md                    # ROS 데이터 계약 제안
│   └── ros2_install_guideline.md     # Windows 설치 및 검증 안내
├── scripts/
│   └── windows/
│       ├── run_rosbridge.py          # Windows 호환 rosbridge 실행 스크립트
│       └── test_rosbridge_ws.ps1     # PowerShell 토픽 왕복·서비스 확인 스크립트
├── src/                              # colcon이 검색하는 ROS 2 패키지
│   ├── control/
│   │   ├── custom_hand_control/      # Custom Hand 제어 패키지 기본 구조
│   │   └── leap_hand_control/        # LEAP Hand 제어 패키지 기본 구조
│   ├── perception/
│   │   ├── README.md
│   │   └── cube_detection/
│   │       ├── cube_detection/detector_node.py # 이미지 구독·YOLO OBB 검출 노드
│   │       ├── cube_detection/cube_state.py # 면 원근 보정·색 분류·상태 검증
│   │       ├── cube_detection/state_assembler_node.py # 오프라인 검증용 6면 상태 조립
│   │       ├── cube_detection/offline_replay_node.py # 저장된 면 이미지 ROS 재생
│   │       ├── cube_detection/camera_capture_node.py # 카메라·수집 통합 및 YOLO 박스 미리보기
│   │       ├── model/
│   │       │   ├── cube-detector.pt   # 파인튜닝 YOLO 가중치
│   │       │   └── .gitkeep
│   │       ├── package.xml
│   │       └── setup.py              # detector·assembler·replay·camera 진입점
│   ├── planning/
│   │   ├── README.md
│   │   └── cube_solver/
│   │       ├── cube_solver/solver_node.py # Kociemba 풀이 서비스 노드
│   │       ├── package.xml
│   │       └── setup.py              # cube_solver 실행 진입점
│   ├── policy/
│   │   ├── README.md
│   │   └── rl_policy/                # 강화학습 정책 패키지 기본 구조
│   ├── system_bringup/               # 시스템 실행·통합 패키지 기본 구조
│   └── system_interfaces/            # 큐브 감지·상태·풀이 인터페이스 패키지
│       ├── CMakeLists.txt            # rosidl 인터페이스 생성 설정
│       ├── package.xml               # 인터페이스 의존성 및 패키지 메타데이터
│       ├── msg/
│       │   ├── FaceDetection.msg     # 큐브 면 OBB 검출 결과
│       │   ├── FaceDetectionArray.msg # 이미지 단위 검출 결과 배열
│       │   ├── FaceCaptureLabel.msg # 이미지와 동기화된 스캔 면 라벨
│       │   ├── CubeState.msg         # URFDLB 큐브 상태 및 검증 결과
│       │   └── CubeSolution.msg      # 큐브 상태와 풀이 수열
│       └── srv/
│           └── SolveCube.srv         # 큐브 풀이 요청·응답
└── log/                              # 현재 남아 있는 이전 colcon 로그

## 현재 상태: Tauri 연결 검증 및 큐브 인식·풀이 연동 구현

- `apps/monitoring_ui`에 Tauri 2 Rust 앱 틀과 정적 HTML/CSS/JavaScript 진단 화면을 추가했습니다. `src`는 ROS 저장소 루트, UI JavaScript, Tauri Rust 코드 각각의 소스 폴더이므로 서로 다른 앱의 영역입니다.
- UI의 WebSocket 주소 기본값은 `ws://127.0.0.1:9090`이며, `/chatter` (`std_msgs/msg/String`) 구독·발행과 `/add_two_ints` (`example_interfaces/srv/AddTwoInts`) 호출 기능을 제공합니다.
- `src-tauri/tauri.conf.json`에는 로컬 rosbridge WebSocket 연결만 허용하는 CSP를 설정했습니다. `scripts/build.ps1`는 별도 Node.js 없이 PowerShell로 Tauri 정적 화면을 `dist/`에 생성합니다.
- rosbridge WebSocket 연결, talker의 `/chatter` 메시지 수신, 발행 메시지 왕복, `17 + 25 = 42` 서비스 응답을 확인했습니다. 이전에는 Node 기반 스크립트로, 현재는 `scripts/windows/test_rosbridge_ws.ps1`로 같은 검증을 통과했습니다. ROS 예제는 `demo_nodes_py talker`와 `demo_nodes_cpp add_two_ints_server`를 사용했습니다.
- Windows 개발 도구 확인 결과 Visual Studio 2026 C++ 빌드 구성 요소와 WebView2가 있고, Rust/Cargo 및 Tauri CLI는 `ROOT\.cargo\bin`에 설치되어 있습니다. PowerShell에서 `cargo`가 바로 인식되지 않으면 새 창을 열거나 Cargo 실행 파일의 전체 경로를 사용합니다.
- Tauri 빌드가 요구하는 `src-tauri/icons/icon.ico`를 추가했고 `cargo tauri dev` 빌드를 통과했습니다. Tauri WebView에서 한국어 진단 화면이 렌더링되는 것을 확인했습니다. 개발 창은 CLI가 제공하는 로컬 정적 서버를 사용하므로 `target/debug` 실행 파일을 따로 열지 말고 `cargo tauri dev`로 실행해야 합니다.
- 사용자가 실제 Tauri 창에서 rosbridge 연결, ROS 토픽, 서비스 통신 검증을 완료했습니다. 앞선 PowerShell WebSocket 검사에 더해 WebView 경로도 확인된 상태입니다.
- `src/system_interfaces`에 `FaceDetection`, `FaceDetectionArray`, `CubeState`, `CubeSolution` 메시지와 `SolveCube` 서비스를 추가했습니다. 좌표는 입력 이미지 픽셀 기준이고, 큐브 facelets는 Kociemba URFDLB 54자 형식을 사용합니다. 손 제어 인터페이스는 실제 드라이버·관절 사양이 정해지지 않아 포함하지 않았습니다.
- Windows에서 Visual Studio 2026의 `v145` 도구 집합과 Pixi Python 개발 파일을 지정해 CMake 구성, 전체 ROSIDL 타깃 빌드, 설치를 확인했습니다. 설치된 인터페이스는 `ros2 interface show`로 조회했습니다. 표준 `colcon build` 실행 경로는 아직 확인되지 않았습니다.
- `cube_detection`은 `sensor_msgs/msg/Image`를 `/camera/image_raw`에서 받아 `/cube/face_detections`로 OBB 결과를 발행합니다. `model/cube-detector.pt`가 패키지 모델 경로에 포함되며 `model_path`, `confidence_threshold`, `device`, 토픽 이름을 매개변수로 설정할 수 있습니다. 기본 추론 장치는 CPU, 기본 confidence 임계값은 0.5입니다. 번들 모델로 참고 디렉토리의 1280×720 샘플 이미지를 처리해 `side` 검출 1개, confidence 0.947 및 꼭짓점 4개가 포함된 ROS 메시지를 확인했습니다.
- `cube_solver`는 `/cube/solve` 서비스를 제공하며 54자 URFDLB 입력, 허용 문자와 면별 9개 개수를 확인한 뒤 `rubik-solver-py==0.1.1`의 Kociemba Two-Phase solver를 호출합니다. 응답에는 성공 여부, 토큰 배열, 실패 사유를 기록합니다. 한 번 회전한 큐브를 푸는 콜백 확인과 assembler 노드에서 실제 서비스 호출을 보내 20수 해법을 받는 통합 검증을 완료했습니다.
- `cube_camera_capture`는 실시간 카메라 경로의 캡처·수집을 통합합니다. 미리보기 프레임을 설정 FPS로 `/camera/image_raw`에 발행하고, 별도 `cube_detector`의 `/cube/face_detections` 응답을 받아 OBB 박스·클래스·신뢰도를 화면에 표시합니다. `Space`로 캡처한 프레임은 해당 timestamp의 검출 결과와 짝지어 색상을 샘플링하며, 여섯 면을 분류·검증해 `/cube/state`를 발행하고 유효하면 `/cube/solve`를 호출합니다. 풀이 성공 시 `/cube/solution` (`CubeSolution`)을 transient-local QoS로 발행하고, `results/cube_solutions/`에 UTC 시각과 scan ID를 포함한 JSON 파일을 저장합니다. `R`은 새 세션으로 초기화합니다. `camera_index`, Windows 캡처 backend (`dshow`, `msmf`, `any`), 프레임 크기, 미리보기 FPS, 검출 FPS 및 JSON 저장 디렉토리를 매개변수로 받습니다. 캡처 순서는 `F, R, B, L, U, D`이며 의미상 방향을 모델이 판별하는 것은 아닙니다.
- `cube_state_assembler`는 `/cube/scan/face_label`을 이용하는 분리형 오프라인 재생·진단 경로에 남아 있으며, 통합 실시간 카메라 절차에서는 실행하지 않습니다.
- `cube_offline_replay`는 시작 후 5초 대기하고 `cube_3D_conversion/captures/{F,R,B,L,U,D}/original.png`를 `/camera/image_raw`와 `/cube/scan/face_label`에 3초 간격으로 두 차례 재생합니다. assembler는 면별 중복 프레임을 무시하고 첫 검출 실패 시 다음 프레임을 재시도합니다. 수동 스캔용 상태 조립기 및 카메라 결과 대기 타임아웃 코드는 `# 자동 인식의 경우 주석 해제` 안내와 함께 주석 처리되어 있습니다. 참조 디렉토리는 읽기 전용이며, 이 재생은 실제 카메라 검증을 대체하지 않습니다.
- 6개 샘플 이미지에 번들 YOLO 모델을 각각 실행하고, 조립된 54자 상태가 참조 `cube_state.json`에서 생성한 URFDLB 문자열과 일치하는 것을 확인했습니다. Kociemba 물리 상태 검증도 통과했습니다. Windows 전체 colcon 빌드와 Python 문법 컴파일, `git diff --check`가 통과했습니다.
- ROS 노드 간 재생 검증 초기에 첫 `F` 프레임이 수신되지 않는 경우를 확인해 재생 시작 전 대기와 두 차례 재생을 적용하고, assembler가 면 중복을 무시하며 실패 입력을 재시도하도록 했습니다. 수정 후 ROS 검증에서 여섯 면 모두 검출·조립됐고, 참조 `cube_state.json`과 같은 54자 URFDLB 상태를 `/cube/state`로 발행했습니다. `/cube/solve` 응답에서 20수 해법을 받았습니다. 검증에 사용한 네 노드는 모두 종료했습니다.
- 사용자가 실행한 Windows 세션에서 `camera_index=0`, `dshow`, 1280×720, 30 FPS 카메라 연결과 여섯 면 캡처·조립이 확인됐습니다. 최초 nearest-center 색상 분류는 색상 개수 `R=11, D=10, L=7, B=8`로 거부됐습니다. 제한된 Lab 거리 보정과 정상 종료 처리가 추가됐고, 이어서 실시간 카메라 캡처·수집 통합과 YOLO OBB 오버레이를 구현했습니다. 통합본의 Windows 재빌드 및 실시간 카메라 재검증은 아직 완료되지 않았습니다.
- 풀이 성공 시 `/cube/solution` 발행과 `results/cube_solutions/` JSON 저장을 구현했습니다. `CubeSolution.msg` 인터페이스 변경 및 저장 기능의 Windows 재빌드·런타임 확인은 아직 완료되지 않았습니다.

### `system_interfaces` 메시지·서비스

`src/system_interfaces`는 `ament_cmake`와 `rosidl_default_generators`로 인터페이스를 생성합니다. `geometry_msgs`와 `std_msgs`를 사용하고 `rosidl_default_runtime`을 내보냅니다. 인터페이스 패키지 그룹은 `rosidl_interface_packages`입니다.

| 정의 | 필드 | 의미 및 제약 |
|---|---|---|
| `FaceDetection.msg` | `class_name: string`, `confidence: float32`, `corners_px: geometry_msgs/Point32[4]`, `center_px: geometry_msgs/Point32` | 한 이미지에서 찾은 큐브 면 OBB. 좌표는 입력 이미지 좌상단 원점의 픽셀 단위이며 `z=0`. 꼭짓점 네 점은 검출기 순서의 다각형 순환 순서입니다. |
| `FaceDetectionArray.msg` | `header: std_msgs/Header`, `detections: FaceDetection[]` | 하나의 원본 이미지에 대응하는 검출 결과 배열. |
| `FaceCaptureLabel.msg` | `header: std_msgs/Header`, `scan_id: string`, `face_name: string` | `F`, `R`, `B`, `L`, `U`, `D` 중 한 면 라벨. `scan_id`는 한 번의 6면 촬영 세션을 식별하며, timestamp로 해당 이미지와 검출 결과를 연결합니다. |
| `CubeState.msg` | `header: std_msgs/Header`, `scan_id: string`, `facelets: string`, `valid: bool`, `observed_faces: string[]`, `validation_errors: string[]` | 유효한 상태의 `facelets`는 Kociemba URFDLB 면 순서의 54자입니다. `scan_id`는 상태를 생성한 촬영 세션입니다. 물리 상태 검증을 통과하지 못했거나 미완성이면 `valid=false`이고 `facelets`는 빈 문자열입니다. `observed_faces`는 중복 없는 `U`, `R`, `F`, `D`, `L`, `B` 목록입니다. |
| `CubeSolution.msg` | `header: std_msgs/Header`, `scan_id: string`, `facelets: string`, `moves: string[]` | 풀이에 사용한 URFDLB 상태와 Kociemba 이동 토큰을 담으며 `/cube/solution`으로 발행합니다. |
| `SolveCube.srv` | 요청 `facelets: string`; 응답 `success: bool`, `moves: string[]`, `error_message: string` | 큐브 상태를 받아 풀이를 반환합니다. 이동은 `R`, `U'`, `F2`와 같은 토큰이며 성공 여부와 오류 메시지를 별도 필드로 전달합니다. |

`cube_detection`은 영상 구독·OBB 검출, 6면 색상 분류 및 `CubeState` 생성을 구현합니다. 저장된 샘플 이미지에서 ROS 토픽과 solver 서비스까지 이어지는 오프라인 경로가 추가되었습니다. 카메라 모델·해상도와 손 하드웨어 세부사항은 타입에 고정하지 않았습니다.

## 확보 모델 및 입출력 조사

세 참조 디렉토리의 소스는 읽기 전용으로 확인했습니다. YOLO 체크포인트는 사용자가 별도 Git 저장소의 모델을 프로젝트에 포함하라고 요청해 `src/perception/cube_detection/model/cube-detector.pt`로 복사했으며, 원본과 SHA-256이 일치합니다. Custom Hand와 LEAP Hand 가중치는 `middleware-bridge`에 포함되어 있지 않습니다. 모델 분석은 체크포인트 메타데이터와 참고 배포 코드에 근거합니다.

### 큐브 YOLO 모델 (`cube_3D_conversion`)

- `cube-detector.pt`는 약 5.9 MB인 Ultralytics YOLO OBB 체크포인트이며 `cube_detection` 패키지 설치 자료로 포함됩니다. 참고 코드가 사용하는 클래스는 `side`이며 프레임에서 큐브 면의 사각형 네 꼭짓점, 중심, 신뢰도를 얻습니다. 모델 입력 크기는 코드에서 직접 지정하지 않아 라이브러리 기본값에 따릅니다.
- 이 모델은 스티커 색상 분류나 3D 큐브 자세 추정, 큐브 풀이를 하지 않습니다. 참고 파이프라인은 검출된 면을 원근 보정하고 3×3 스티커의 BGR 색을 추출한 다음, 여섯 면을 모아 검증된 54칸 큐브 상태를 구성합니다. 이후 Kociemba URFDLB 문자열로 바꾸고 풀이 결과를 생성·검증해 `cube_state.json`, `solution.json`으로 저장합니다.
- 따라서 ROS 데이터 흐름에서는 이미지 입력, OBB 검출 결과, 완성된 큐브 상태, 풀이 결과가 서로 다른 단계입니다. 카메라 해상도·모델 입력 크기·캡처 시퀀스는 아직 본 프로젝트에 확정되지 않았습니다.

### Custom Hand 정책 (`left-rl-with-mujoco`)

- 확인한 PPO 파일은 `model/finger_robot_ppo_best.zip`(673,505 학습 스텝), `model/finger_robot_ppo_final.zip`(1,001,472 스텝), `sim2real/finger_robot_ppo_best.zip`(197,306 스텝)입니다. 모두 SB3 PPO 체크포인트이며 정적 메타데이터상 관측 8차원, 행동 4차원입니다. 서로 다른 파일이며 해시도 다르므로 이 중 어느 것이 통합 대상인지는 선택되지 않았습니다.
- `sim2real` 배포 코드는 관측을 네 텐던 위치와 네 속도로 구성하고, 네 개의 정규화 행동값([-1, 1])을 네 모터의 목표 위치로 변환하는 흐름을 설명합니다. 명목 정책 주기는 30 ms입니다. 참고 설정의 모터 순서는 right/front/left/back 및 ID 4/1/2/3으로 되어 있지만, 실제 프로젝트의 손 하드웨어와 보정값으로 간주하지 않습니다.
- 이 정책은 4개 구동 입력을 사용하는 Custom Hand 모델입니다. LEAP Hand의 16관절 모델과 같은 인터페이스로 취급할 수 없습니다. 8차원 관측을 만드는 센서·보정 정의와 사용할 체크포인트를 먼저 확정해야 합니다.

### LEAP Hand 정책 (`right_rl_with_mujoco`)

- 배포 산출물 `policy/policy_np.npz`는 108개 평균·표준편차 값과 108→512→256→128→32 가중치를 포함하며, 정책 행동 크기는 16입니다. 정적 검사에서 정규화 통계는 유한값이었습니다. 배포 코드는 정규화 후 ReLU MLP와 tanh를 적용해 행동을 계산합니다. 별도 `policy/distilled_policy` 파일은 약 898 KB의 증류 정책 산출물입니다.
- 배포 관측 조립 코드는 16개 관절각, 관절 오차 이력, 큐브 위치 오차 이력, 큐브 방향 오차 이력, 직전 행동 및 방향 오차를 합쳐 108차원을 구성합니다. 행동은 16개 [-1, 1] 값이며, 참고 실행 코드는 이를 0.2 rad 스케일의 관절각 증분으로 누적합니다. 이는 큐브 면 방향을 재배치하는 정책이며, 큐브 풀이 수열을 수행하는 정책과는 역할이 다릅니다.
- 참고 실행 코드는 카메라 기반 큐브 자세를 요구하고 마커·카메라 보정 자료를 사용합니다. 현재 프로젝트의 카메라 정보와 LEAP Hand 하드웨어 세부사항은 미정이므로 이 관측을 실제 ROS 토픽으로 구성하거나 행동을 하드웨어에 전송하는 것은 아직 정할 수 없습니다. `BEST_sac_checkpoint.txt` 및 교사 체크포인트 표기 경로는 Windows 참조 디렉토리에 존재하지 않으며, 배포용 정책 파일은 별도로 존재합니다.

참고용 디렉토리 `cube_3D_conversion`, `left-rl-with-mujoco`,
`right_rl_with_mujoco`는 이 프로젝트 트리 외부에 있으며 빌드 입력으로
사용하지 않습니다. YOLO 체크포인트는 `middleware-bridge` 안에 복사되어 있고,
Custom Hand와 LEAP Hand 가중치는 참조 디렉토리에만 있습니다. ROS 이미지 메시지를
YOLO 노드에 전달한 샘플 검증은 완료했습니다. 실제 카메라가 여섯 면을 수집하는 것도
확인했으며, 최초 실시간 분류 결과는 색상 개수 검증에서 거부되어 제한된 색상 보정과
카메라 종료 처리를 수정했습니다. 수정본의 실제 카메라 재검증은 아직입니다.

Pixi와 colcon 출력은 로컬에서 생성됩니다. `.pixi/`, `.colcon-win/`,
`install/`, `log/`는 Git 추적 대상에서 제외되며, Windows 빌드 출력 위치는
`.colcon-win/`입니다. 트리에 표시된 루트 `log/`는 작업 디렉토리에 남아 있는
이전 생성물입니다.
