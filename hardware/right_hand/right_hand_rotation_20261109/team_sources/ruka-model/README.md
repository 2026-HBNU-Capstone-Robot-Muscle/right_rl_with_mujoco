# 🦾 RUKA Hand Control & Simulation Model (`ruka-model`)

`ruka-model` 프로젝트는 3D 다관절 로봇 핸드인 **RUKA Hand**의 제어, 시뮬레이션 환경(MuJoCo) 구축, 그리고 원격 조종(Teleoperation) 및 학습 데이터셋의 통계 연산을 지원하기 위한 오픈소스 저장소입니다.

이 가이드는 프로젝트의 디렉토리 구조와 주요 스크립트의 기능, 사용 방법을 상세하게 설명합니다.

---

## 📂 리포지토리 디렉토리 구조 (Repository Directory Structure)

```text
ruka-model/
├── 📁 assets/                     # 시뮬레이션 모델 자산 및 데모 미디어
│   ├── 📁 xml/                    # MuJoCo XML 환경 설정 및 시뮬레이션 씬 파일
│   │   ├── 📁 meshes/             # 로봇 핸드 3D CAD 메쉬 파일 (STL 등)
│   │   ├── 📁 reorientation_cube_textures/ # 큐브 재배치 작업용 텍스처 파일
│   │   ├── 📄 hand_assembly.xml   # RUKA Hand의 메인 조립체 정의
│   │   ├── 📄 hand_basic_scene.xml# 기본 배경 및 조명 환경 씬
│   │   ├── 📄 hand_box_scene.xml  # 상자 조작 테스트 씬
│   │   └── 📄 reorientation_cube.xml # 큐브 조작 환경 정의
│   ├── 🖼️ architecture.png       # 시스템 아키텍처 다이어그램
│   ├── 📹 calibration.gif         # 캘리브레이션 동작 가이드
│   ├── 📹 human_eval.gif         # 인간 모션 데이터를 이용한 로봇 손 제어 시연
│   ├── 📹 robot_eval.gif         # 로봇 자체의 태스크 수행 평가 시연
│   └── 📹 ruka.gif                # RUKA 로봇 구동 데모
├── 📁 sample_code/                # 예제 코드 및 동작 제어/유틸리티 스크립트
│   ├── 📁 examples/               # 컨트롤러 테스트 예제
│   │   └── 🐍 test_controllers.py # RUKAOperator 컨트롤러 실행 데모
│   └── 📁 scripts/                # 하드웨어 세팅 및 데이터 처리 헬퍼 스크립트
│       ├── 🐍 calculate_dset_stats.py # Hydra 기반 학습 데이터셋 통계 산출
│       └── 🐍 reset_motors.py     # 로봇 손 모터 초기 텐션 위치로 부드럽게 초기화
└── 📄 README.md                   # 본 문서
```

---

## 🛠️ 핵심 구성 요소 설명

### 1. [assets](./assets) (시뮬레이션 모델 & 미디어)
*   **MuJoCo 시뮬레이션 환경 (`xml/`):**
    *   [hand_assembly.xml](./assets/xml/hand_assembly.xml): RUKA Hand의 관절(Joint), 액추에이터(Actuator) 및 링크 정보를 담은 주요 MuJoCo 모델 파일입니다.
    *   [hand_basic_scene.xml](./assets/xml/hand_basic_scene.xml), [hand_box_scene.xml](./assets/xml/hand_box_scene.xml), [reorientation_cube.xml](./assets/xml/reorientation_cube.xml): 다양한 환경 시나리오와 물체 상호작용을 테스트하기 위한 씬 구성 파일입니다.
*   **데모 시각 자료:**
    *   로봇 손 캘리브레이션([calibration.gif](./assets/calibration.gif)), 사람 손 매핑 평가([human_eval.gif](./assets/human_eval.gif)), 실제 구동 영상([ruka.gif](./assets/ruka.gif)) 등이 포함되어 시뮬레이션 결과 및 작업 흐름을 파악하기 좋습니다.

### 2. [sample_code](./sample_code) (동작 예시 및 유틸리티)
*   **[test_controllers.py](./sample_code/examples/test_controllers.py):**
    *   `ruka_hand` 라이브러리의 `RUKAOperator` 클래스를 활용하여 사전 저장된 인간/로봇의 keypoint 좌표(`.npy`)를 기반으로 로봇 컨트롤러 루프를 주기적으로 실행하는 예제 코드입니다.
*   **[calculate_dset_stats.py](./sample_code/scripts/calculate_dset_stats.py):**
    *   모델 학습 이전에 데이터셋의 통계값(Mean, Variance 등)을 계산해 `dataset_stats.pkl`로 저장합니다. `Hydra` 및 `OmegaConf` 설정을 동적으로 불러와 데이터 가공을 간소화합니다.
*   **[reset_motors.py](./sample_code/scripts/reset_motors.py):**
    *   로봇 핸드 제어 모터들을 물리적으로 텐션이 완료된 기본 홈 위치(`hand.tensioned_pos`)로 부드럽게 복구(리셋)시켜주는 안전 헬퍼 스크립트입니다.

---

## 🚀 가이드 및 사용법 (How to Use)

### 📌 1. 필수 라이브러리 및 환경 구축
해당 패키지를 작동시키기 위해 Python 환경과 필요한 의존 모듈들을 설치해야 합니다.

```bash
# ruka_hand 코어 모듈이 설치되어 있어야 합니다.
pip install torch omegaconf hydra-core numpy
```

> [!NOTE]
> `ruka_hand` 커스텀 라이브러리는 하드웨어 통신과 모델 핵심 연산 기능을 내포하고 있어야 정상 작동합니다.

### 📌 2. 모터 초기화 및 텐션 복구 (하드웨어 준비)
로봇 조작 전, 안전하게 손의 모터 관절 텐션을 원상 복귀하고자 할 때 아래 스크립트를 실행합니다.

```bash
# 기본값으로 'left' 핸드 설정이 리셋됩니다.
python sample_code/scripts/reset_motors.py

# 'right' 핸드를 지정해 모터를 초기화하려면 아래 옵션을 사용합니다.
python sample_code/scripts/reset_motors.py --hand_type right
```

### 📌 3. 데이터셋 통계 계산 (학습 준비)
기존 학습 진행 정보가 담긴 특정 디렉토리 경로에 대하여 전체 데이터셋의 통계 처리를 하고자 할 경우:

1. [calculate_dset_stats.py](./sample_code/scripts/calculate_dset_stats.py) 코드 내 `training_paths` 변수에 해당 모델 체크포인트 혹은 실험 폴더 경로를 지정합니다.
2. 스크립트를 실행합니다.
   ```bash
   python sample_code/scripts/calculate_dset_stats.py
   ```
3. 연산 완료 후 해당 실험 경로 내에 `dataset_stats.pkl` 파일이 생성됩니다.

### 📌 4. 컨트롤러 예제 구동 (동작 제어)
키포인트를 기반으로 제어 연산자가 어떻게 동작하는지 보려면 다음 예제를 확인합니다.

```bash
python sample_code/examples/test_controllers.py
```
*   `ruka_data/osfstorage/examples/human_examples_right.npy` 등에서 모션 키포인트를 읽어 `FrequencyTimer`에 의해 주기적(예: 10Hz)으로 RUKA Operator를 구동하는 전체 구조를 시뮬레이션하거나 실 기기에 전송할 수 있습니다.

---

## 🎨 시각 자료 및 시스템 시연 (Visual Demonstration)

### 시스템 아키텍처
![Architecture](./assets/architecture.png)

### RUKA Hand 동작 데모
| 캘리브레이션 | 모션 평가 | 로봇 조작 |
| :---: | :---: | :---: |
| ![Calibration](./assets/calibration.gif) | ![Human Eval](./assets/human_eval.gif) | ![Robot Eval](./assets/robot_eval.gif) |
