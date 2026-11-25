# train — residual SAC + DAgger 증류

## 무엇이 무엇인가

`run_residual_distill.sh` 는 **순서를 지시하는 스크립트**이지 알고리즘이 아닙니다.
실제 학습 코드는 `learning/` 에 있습니다.

| 파일 | 역할 |
|---|---|
| `learning/train_jax_sac.py` | SAC 학습 본체. `--residual_teacher` 를 주면 래퍼를 끼운다 |
| `learning/residual_wrapper.py` | `a = clip(a_교사(s) + scale·a_학생(s), -1, 1)` |
| `learning/distill_residual.py` | DAgger 증류. 교사+residual 결합 정책 -> 단일망 |
| `learning/eval_chain.py` | 연쇄 평가. `--residual_teacher` 로 **합성해서** 평가 |
| `learning/render_policy.py` | 영상 |
| `run_faceup_sac.sh` | SAC 실행기 (하이퍼파라미터·오버라이드 조립) |
| `run_residual_distill.sh` | [1]학습 → [2]평가·선택 → [3]증류 → [4]평가 → [5]내보내기 |
| `env/` | **상류 mujoco_playground 에 없는 환경 수정** |

## env/ 가 왜 필요한가

학습 코드만으로는 돌지 않습니다. 과제 2 는 상류에 없는 것들에 의존합니다.

- `face_up.py` — 과제 2 환경 자체(면 맞추기 + 제자리 복귀). 상류에 없습니다.
- `reorient.py` — 연쇄 목표(`chain_goals`), `settle_steps`, `hyperext_floor`,
  `action_delay_steps`.
- `leap_rh_mjx.xml` — **엄지 관절 범위 교정.** menagerie 원본은 `th_axl`/`th_mcp`
  가 한 칸 밀려 있어 `th_mcp` 상한이 실제 스토퍼보다 31도 더 굽혀졌습니다.
  공식 URDF 및 LEAP SDK 의 `LEAPsim_limits` 와 대조해 고친 것입니다.

이 셋이 빠지면 학습은 돌지만 **다른 과제, 다른 손**을 학습합니다.

## 설치

```bash
git clone https://github.com/google-deepmind/mujoco_playground.git
cd mujoco_playground && pip install -e .
pip install "jax[cuda12]" mediapy imageio imageio-ffmpeg

PG=/path/to/mujoco_playground ./setup_env.sh
```

원본은 `.bak_<타임스탬프>` 로 남깁니다. 끝에 환경을 실제로 로드해
`hyperext_floor` 까지 먹었는지 확인합니다.

## 실행

```bash
PG=/path/to/mujoco_playground \
TEACHER=/path/to/ppo_checkpoint_dir \
OUT=./out \
./run_residual_distill.sh
```

`TEACHER` 는 **체크포인트 디렉터리**입니다. 특정 스텝 폴더를 주면
`ValueError: invalid literal for int()` 가 납니다.

`policy/TEACHER_checkpoint.txt` 에 이번에 쓴 교사 경로가 적혀 있습니다.

## 단계

| | | 소요 |
|---|---|---|
| [1] | residual SAC 2시드 x 40M | 시드당 약 1.6시간 (RTX 5090) |
| [2] | 합성 평가 6건 → 최고점 선택 | 약 25분 |
| [3] | DAgger 증류 (토크 4구간) | 약 70분 |
| [4] | 증류·교사 평가 (토크 3구간) | 약 25분 |
| [5] | `policy/policy_np.npz` 내보내기 | 1분 |

## 이 파이프라인에서 실제로 겪은 함정

**평가에서 교사를 빼먹으면 안 됩니다.** 학생은 교정량만 내도록 학습되므로,
저장된 SAC 체크포인트를 단독으로 굴리면 0.1배 교정량이 절대 명령이 되어 손이
멈춥니다(실측: 연쇄 0.00, 낙하 96.1%). 그러면 6개 체크포인트가 전부 0 을 받아
**최고점 선택이 사실상 무작위**가 됩니다. `eval_chain.py --residual_teacher` 로
합성해야 합니다.

**연쇄 설정을 학습·평가·증류에 전부 같이 넘겨야 합니다.** `chain_goals`,
`hold_steps`, `settle_steps`, `hyperext_floor` 중 하나라도 어긋나면 교사와
학생이 다른 과제를 풉니다. 증류는 DAgger 라 학생이 방문할 상태 분포에서
라벨을 받아야 하는데, 여기가 어긋나면 전제가 깨집니다.

**residual 에서는 시연 혼합을 쓰지 않습니다.** 시연의 action 은 절대 행동인데
residual 환경에서는 교정량으로 해석되어 critic 이 Q(s,a) 를 잘못된 의미로
배웁니다. `run_faceup_sac.sh` 가 residual 모드에서 강제로 끕니다.

## 결과 (참고)

| 토크 (N·m) | 교사 팔 F | 증류망 |
|---|---|---|
| 0.11 | 7.19 | 5.98 |
| 0.16 | 12.50 | 10.89 |
| 0.21 | 14.87 | 13.52 |

residual SAC 은 2시드 x 3체크포인트 6개 전부가 교사에 못 미쳤습니다(6.52~6.89).
증류망도 모든 토크에서 교사보다 낮습니다. 증류의 값은 성능이 아니라
**단일망 · 토크 강건성 · SAC 요건**입니다.

**토크가 지배적입니다.** 0.11 → 0.16 에서 74% 오릅니다. 실기 `curr_lim=450mA`
가 0.165 N·m 이므로 **0.165 로 재학습하는 것이 지금 가장 값싼 개선**입니다.
`FORCE=0.165` 로 넘기면 됩니다.
