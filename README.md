# Leap_Test — LEAP Hand 큐브 면 맞추기 실기 패키지

과제 2(LeapCubeFaceUp)의 실기 이관·평가에 필요한 것을 모았습니다.
학습 저장소(`캡스톤_강화학습SAC`)와 분리되어 있어 **이 폴더만 실기 PC 로
복사하면 됩니다.**

자세한 내용은 `docs/실기이관_평가_가이드.pdf` (15쪽)를 보십시오.

다른 AI 세션이 이어받을 때는 **`AI_session_Log.md`** 를 먼저 읽으십시오 —
결정과 근거, 실측 수치, 반복하면 안 되는 함정이 정리돼 있습니다.

## 설치 (실기 PC)

```bash
pip install numpy opencv-contrib-python dynamixel_sdk
```

jax·brax·orbax 는 **필요 없습니다.** 정책이 numpy 로 돕니다.

`deploy/leap_hand_utils/dynamixel_client.py` 를 함께 넣어 두었습니다(LEAP Hand SDK).
**전송 계층만 재사용하고 `set_ones`/`set_allegro` 같은 규약 변환 함수는 쓰지
않습니다** — 다른 좌표계를 가정하기 때문입니다.

## 반드시 재서 넣어야 하는 값 두 개

`vision/rubik_center/track_cube.py` 상단에 있습니다. 기본값으로 두면 **위치 관측이 그만큼
틀립니다.** 성공 임계값이 15 mm 라 무시할 수 없습니다.

| | 뜻 | 기본값 |
|---|---|---|
| `PALM_IN_MARKER` | 손 마커 프레임에서 본 손바닥(grasp_site) 위치 [m] | `0,0,0` |
| `HAND_TO_WORLD` | 손 마커 프레임 → 시뮬 월드 회전 | 단위행렬 |

`PALM_IN_MARKER` 를 0 으로 두면 마커 위치를 손바닥으로 간주하는 것입니다.
자로 재서 넣으십시오. `HAND_TO_WORLD` 는 마커를 손등에 수평으로, u 축을 시뮬
+x, v 축을 +y 로 맞춰 붙이면 단위행렬 그대로입니다.

## 순서

| 단계 | 명령 | 확인할 것 |
|---|---|---|
| 1 | `python deploy/check_joints.py --mode map` | 관절 매핑 |
| 2 | `python deploy/check_joints.py --mode sign` | 외전 3개 부호 |
| 3 | `python deploy/check_joints.py --mode latency` | 총 지연 |
| 4 | `python vision/rubik_center/make_center_templates.py` | 마커 인쇄 |
| 5 | `python vision/rubik_center/calibrate_camera.py "calib0/*.jpg" camera0.npz` | 재투영 < 0.5px |
| 6 | 면별 마커 회전 추정 (`calibrate_face_rot`) | 재투영 < 1px |
| 7 | `python deploy/policy_runner.py --cams ...` | 루프 시간, 끊김률 |
| 8 | `... --chain` | **연속 달성 횟수** (시뮬 지표와 직접 비교) |

## 배포 산출물

`policy/policy_np.npz` — 가중치 + **정규화 통계**. 통계를 빠뜨리면 정책은
무의미한 출력을 냅니다. 이 파일 하나에 둘 다 들어 있습니다.

## 검증된 것 (실기 없이)

| 항목 | 결과 |
|---|---|
| 관측 조립기 vs 시뮬 `_get_obs` | 최대 오차 **1.7e-07** (12스텝) |
| numpy 추론 vs jax 추론 | 최대 오차 **7.6e-06** (관측 512개) |
| 드라이버 경계면 왕복 | 항등, 순열이 유효한 치환 |
| 마커 시트 | 25 / 24 / 19개 전부 정확한 ID 로 재검출 |
| 큐브 보드 기하 | 6면 법선·평면성·변 길이 통과 |
| 면 회전 보정 | 합성 잡음 1px 까지 참값 복원 |
| 행렬-쿼터니언 변환 | 왕복 오차 3.3e-16 |
| 카메라→월드 프레임 변환 | 카메라 위치와 무관, 두 대 간 불일치 1e-16 |
| 제어 루프 구성요소 100스텝 | 관측 108차원 유한, 명령 범위 유지, **0.19 ms/스텝** |
| 목표 자세 6개 vs 면 규약 | 6면 전부 일치 |

```bash
python tests/test_obs_match.py     # 학습 저장소의 venv 에서 (jax 필요)
python tests/test_frames.py        # opencv 필요
python tests/test_loop_smoke.py    # numpy 만
```

## 검증되지 않은 것

**하드웨어가 있어야 확인되는 것들입니다.**

- `if_rot`·`mf_rot`·`rf_rot` 의 **부호** — 범위가 ±1.047 로 대칭이라 범위 대조로
  못 가렸습니다. 틀리면 굽힘과 벌림이 통째로 바뀝니다. 1단계에서 최우선.
- 엄지 4관절 매핑
- `track_cube.py` / `multi_camera.py` / `calibrate_camera.py` — 카메라 필요
- `leap_hw.py` 의 서보 I/O — 실기 필요
- 전원·전류 병목 (`test_sequential_load.py` 계열)

## 알아둘 것

**토크가 지배적입니다.** 교사 기준 0.11 → 0.16 N·m 에서 연속 달성이 7.19 → 12.50
으로 74% 오릅니다. 실기 `curr_lim = 450 mA` 는 0.165 N·m 에 해당하므로,
지금까지의 0.11 기준 수치는 실기 성능의 하한에 가깝습니다.

**증류망은 교사보다 모든 토크에서 낮습니다** (−9 ~ −17%). 증류의 값은 성능이
아니라 단일망·토크 강건성·SAC 요건입니다. 성능만 본다면 교사 PPO 배포가 낫습니다.

## 재학습

```bash
TEACHER=<PPO 체크포인트 디렉터리> ./train/run_residual_distill.sh
```

[5] 단계가 `policy/policy_np.npz` 를 자동으로 갱신합니다.
