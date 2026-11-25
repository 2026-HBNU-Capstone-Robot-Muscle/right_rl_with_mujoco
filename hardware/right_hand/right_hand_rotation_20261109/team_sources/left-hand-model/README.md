# left-hand-model (좌수 대칭형 4지 로봇 핸드 설계)

본 저장소는 좌수 대칭형 4지 로봇 핸드(Left-hand Symmetric 4-finger Robot Hand)의 CAD 파트 STL 파일 및 ROS 시각화/시뮬레이션을 위한 URDF 모델 패키지를 포함하고 있습니다.

![전체 조립 형상](./Figure/Outline.png)

---

## 📌 주요 특징
- **좌수 대칭형 디자인**: 왼손 형태의 대칭 구조로 설계된 4지 로봇 핸드
- **시각화 및 충돌 검사 최적화**: 
  - **Visual Mesh**: 액추에이터 케이스, 커넥터, 볼트류를 포함하여 실제와 동일하게 렌더링된 메쉬 제공
  - **Collision Mesh**: 시뮬레이션 및 충돌 감지 속도를 향상시키기 위해 볼트나 세부 곡면을 단순화한 메쉬 제공
- **ROS 호환**: RViz를 통해 손쉽게 링크 관계 및 구동 상태를 확인할 수 있는 `.launch` 및 `.urdf` 파일 내장

---

## 📂 디렉토리 구조

```directory
left-hand-model/
├── Figure/
│   └── Outline.png                  # 전체 조립 형상 이미지
├── mesh/
│   ├── part/                       # 각 링크별 원본 및 단순화 파트 STL 파일
│   │   ├── base/                   # 핸드 베이스 파트 (collision, visual)
│   │   ├── link1/                  # 링크 1 파트 (collision, visual)
│   │   ├── link2/                  # 링크 2 파트 (collision, visual)
│   │   ├── pulley/                 # 풀리 파트 (visual)
│   │   └── silicone-joint/         # 실리콘 관절 파트 (collision, visual)
│   │
│   ├── stl/                        # Onshape 어셈블리에서 내보낸 STL 파일 그룹
│   │   ├── collision-mesh/         # 단순화된 충돌 검사용 어셈블리 STL 파일들
│   │   └── visual-mesh/            # 상세 조립 형상 시각화용 어셈블리 STL 파일들
│   │
│   └── urdf/                       # ROS 연동을 위한 URDF 패키지 구성
│       ├── collision-mesh/
│       │   ├── launch/             # ROS 1 론치 파일 (collision_mesh_assembly.launch)
│       │   ├── meshes/             # URDF에서 참조하는 충돌 메쉬 STL 파일들
│       │   └── urdf/               # 충돌 모델 URDF (collision_mesh_assembly.urdf)
│       └── visual-mesh/
│           ├── launch/             # ROS 1 론치 파일 (visual_mesh_assembly.launch)
│           ├── meshes/             # URDF에서 참조하는 상세 시각화 메쉬 STL 파일들
│           └── urdf/               # 상세 시각화 모델 URDF (visual_mesh_assembly.urdf)
└── README.md                       # 본 문서
```

---

## ⚙️ 하드웨어 구성 요소 및 메쉬 상세

### 1. 개별 파트 (`mesh/part`)
실제 가공/출력이나 타 시뮬레이터 포팅에 적합한 개별 파트 메쉬 파일 리스트입니다.

| 분류 | 파트 명칭 | Visual 메쉬 | Collision 메쉬 | 설명 |
| :--- | :--- | :---: | :---: | :--- |
| **Base** | 핸드 베이스 | [`base_visual.stl`](./mesh/part/base/base_visual.stl) | [`base_collision.stl`](./mesh/part/base/base_collision.stl) | 4지 로봇 핸드의 주 지지체 |
| **Link 1** | 첫 번째 손가락 링크 | [`link1_visual.stl`](./mesh/part/link1/link1_visual.stl) | [`link1_collision.stl`](./left-hand-model/mesh/part/link1/link1_collision.stl) | 구동 링크 1 |
| **Link 2** | 두 번째 손가락 링크 | [`link2_visual.stl`](./mesh/part/link2/link2_visual.stl) | [`link2_collision.stl`](./mesh/part/link2/link2_collision.stl) | 구동 링크 2 |
| **Pulley** | 구동 풀리 | [`pulley_visual.stl`](./mesh/part/pulley/pulley_visual.stl) | - | 구동용 와이어 연동 풀리 |
| **Joint** | 실리콘 유연 관절 | [`joint_visual.stl`](./mesh/part/silicone-joint/joint_visual.stl) | [`joint_collision.stl`](./mesh/part/silicone-joint/joint_collision.stl) | 손가락 간의 유연성 및 복원력을 위한 관절부 |

### 2. URDF용 상세 비주얼 메쉬 (`mesh/urdf/visual-mesh/meshes`)
- 온셰이프(Onshape)에서 자동 생성된 파트 외에도 실제 조립에 필요한 상세 부품들이 포함되어 있습니다.
- **액추에이터 DUMMY (DC15_A01 시리즈)**: 케이스 전/중/후면 프레임 및 혼(Horn) 모델링 포함
- **체결 부품**: PH 볼트 스크류 (`_92000A013...`)
- **전기 커넥터**: `B3B_EH.stl`

---

## 🚀 ROS 실행 가이드

본 패키지의 URDF는 ROS 1 환경에서 바로 실행 및 시각화할 수 있도록 지원합니다.

> [!NOTE]
> ROS 구동 전, 본 패키지가 워크스페이스(`src/` 폴더 하위)에 빌드되거나 `ROS_PACKAGE_PATH`에 등록되어 있어야 URDF의 `package://` 경로 탐색이 올바르게 작동합니다.

### 1. 상세 시각화 모델 실행 (Visual Model)
액추에이터 하우징, 스크류 등 실제 형상이 모두 표현되는 모델을 RViz에서 확인합니다.

```bash
roslaunch visual_mesh_assembly visual_mesh_assembly.launch
```

### 2. 충돌 검사용 모델 실행 (Collision Model)
시뮬레이터 물리 엔진 및 모션 플래닝에서 사용되는 단순화된 모델을 RViz에서 확인합니다.

```bash
roslaunch collision_mesh_assembly collision_mesh_assembly.launch
```

---

## 🛠️ 모델링 수정 및 재설계 가이드
- 본 모델링 데이터는 온셰이프(Onshape)를 통해 설계 및 ROS URDF Exporter로 빌드되었습니다.
- 관절 자유도 조절이나 모션 테스트를 원하시는 경우 `joint_state_publisher_gui`를 통해 실시간으로 각 관절 축을 움직여 볼 수 있습니다.



## 스크립트

관절이 고정된 visual_mesh 파일 확인
```
# 상세 비주얼 모델 변환 및 실행
.\isaaclab.bat -p scripts\tools\convert_urdf.py ..\left-hand-model\mesh\urdf\visual-mesh\urdf\visual_mesh_assembly.urdf ..\left-hand-model\mesh\urdf\visual-mesh\urdf\visual_mesh_assembly.usd --merge-joints
```

관절이 고정된 collision_mesh 파일 확인
```
.\isaaclab.bat -p scripts\tools\convert_urdf.py ..\left-hand-model\mesh\urdf\collision-mesh\urdf\collision_mesh_assembly.urdf ..\left-hand-model\mesh\urdf\collision-mesh\urdf\collision_mesh_assembly.usd --merge-joints
```

