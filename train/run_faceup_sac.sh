#!/usr/bin/env bash
# 과제 2 (LeapCubeFaceUp) SAC 학습 — 계획서상 주 알고리즘.
#
# 과제 1 에서 확정된 레시피를 그대로 옮긴다:
#   A1: 시연을 온라인 버퍼와 분리해 읽기 전용으로 두고 배치의 25% 를 거기서 뽑는다.
#       (온라인 버퍼에 부어 넣으면 FIFO 라 40M 중 앞 10% 에서 전부 밀려난다)
#   B1: actor 손실에 시연 모방 MSE 를 mean|Q| 로 정규화해 더한다 (bc_coef=0.4).
# 과제 1 에서 A1 이 +38.5%p, B1 이 +4.2%p 였고 나머지 축(C1/E1/앙상블/Q-filter)은
# 전부 시드 분산에 묻혔으므로 여기서는 두 축만 쓴다.
#
# hold_gated_success 는 끈다(False). 게이트는 PPO 의 보상 착취를 막지만 SAC 에서는
# 조밀 신호까지 없애 오히려 해로웠다(과제 1 v31: 36.8% -> 27.8%).
set -uo pipefail

# 경로는 환경변수로 받는다. Leap_Test 로 옮겨도 동작하도록 한 것이다.
#   PG   : mujoco_playground 저장소 루트 (learning/ 이 있는 곳)
#   ROOT : 산출물이 쌓일 곳
ROOT=${ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}
PG=${PG:?PG 를 mujoco_playground 저장소 루트로 지정하세요}
source ~/rl/bin/activate
cd "$PG"

SUFFIX=${SUFFIX:?SUFFIX 를 지정할 것}
OUTDIR=${OUTDIR:-$ROOT/sac_$SUFFIX}
STEPS=${STEPS:-40000000}
EVALS=${EVALS:-11}
MINREPLAY=${MINREPLAY:-200000}
SEED=${SEED:-1}
THRESHOLD=${THRESHOLD:-0.15}
POS_THRESHOLD=${POS_THRESHOLD:-0.015}
# 콜론 없는 형태여야 한다. ${DEMO:-...} 는 **빈 문자열도** 기본값으로 바꾸므로
# DEMO="" 로 BC 를 끄려던 호출이 조용히 기본 시연(70mm 큐브 시절 것)을 쓴다.
# 실제로 v30 에서 그렇게 되어 세 팔이 전부 무효가 됐다.
DEMO=${DEMO-$ROOT/demo_faceup/buffer}
if [ "${DEMO:-}" = "$ROOT/demo_faceup/buffer" ]; then
  echo "경고: DEMO 기본값(demo_faceup, 2026-08-16 생성=70mm 큐브)을 쓰고 있다." >&2
fi
DEMO_RATIO=${DEMO_RATIO:-0.25}
BC_COEF=${BC_COEF:-0.4}
LOAD_CKPT=${LOAD_CKPT:-}
LR=${LR:-3e-5}
# 유지 완주 유인 관련 노브. 진단 결과 실패가 전부 "간발의 차"(유지 5~8/10)라
# 완주 유인이 부족한 것이 유력하다.
HOLD_BONUS=${HOLD_BONUS:-0.0}     # 스텝당 보상 유지 + 달성 시 일시금
GATED=${GATED:-false}             # true 면 스텝당 보상 없이 달성 일시금만
HOLD_STAIRS=${HOLD_STAIRS:-1}     # 게이트 일시금 계단 수 (전파 거리 H/n 로 축소)
RESET_BEST=${RESET_BEST:-false}   # best_ori_error 에피소드 초기화 (best_progress 복구)
# 도메인 랜덤화. MuJoCo Playground 논문의 LEAP 프로토콜은 이걸 켜고 학습하는데
# 우리는 지금까지 한 번도 켠 적이 없다. 랜덤화 함수는 이미 구현·등록돼 있다
# (마찰·큐브질량/무게중심·qpos0·정지마찰·armature·링크질량·kp/kd).
DOMAIN_RAND=${DOMAIN_RAND:-false}
SIMBA=${SIMBA:-false}             # SimBa 구조 네트워크 (ICLR 2025)
# critic LayerNorm. **이미 켜져 있다** — playground 의 LeapCube* SAC 설정이
# q_network_layer_norm=True 를 기본값으로 준다(brax 함수 기본값 False 와 헷갈리기
# 쉽다). 따라서 이 노브는 켜는 용도가 아니라 끄고 재보는 절제 실험용이다.
# 빈 값이면 설정 기본값(=True)을 그대로 쓴다.
Q_LAYER_NORM=${Q_LAYER_NORM:-}
# 할인율. 기본 0.99 는 유효 지평 100스텝(5초)인데 에피소드는 800스텝(40초)이고
# GATED=true 면 성공 보상이 유지 100스텝을 채운 뒤에야 한 번 나온다.
# 0.99^100=0.366 vs 0.997^100=0.741.
DISCOUNTING=${DISCOUNTING:-0.99}
SIMBA_HIDDEN=${SIMBA_HIDDEN:-128} # 128 이면 기준선(512-256-128)과 파라미터 수 비슷
NUM_EVAL_ENVS=${NUM_EVAL_ENVS:-128}
MAX_REPLAY=${MAX_REPLAY:-2000000}
HOLD_STEPS=${HOLD_STEPS:-10}      # 학습 시 요구 유지 스텝(평가는 항상 10)
SUCCESS_REWARD=${SUCCESS_REWARD:-75.0}
JOINT_MARGIN=${JOINT_MARGIN:-0.0}  # 모터 보호
FORCE_LIMIT=${FORCE_LIMIT:-0.0}    # 모터 토크 제한(Nm). 0.11=300mA, 0.19=550mA
# cd "$PG" 이후에 해석되면 상대 경로가 깨지므로 먼저 절대 경로로 굳힌다.
[ -n "$LOAD_CKPT" ] && LOAD_CKPT=$(readlink -f "$LOAD_CKPT")
# 시연 경로가 없으면 readlink 가 빈 문자열을 돌려주고, 그러면 --demo_data_path
# 가 통째로 빠져 "시연 없는 SAC"가 조용히 학습된다. 실제로 그렇게 2시간을
# 날린 적이 있으므로 여기서 크게 실패시킨다.
if [ -n "$DEMO" ]; then
  if [ ! -e "$DEMO" ]; then
    echo "시연 버퍼가 없다: $DEMO" >&2; exit 1
  fi
  DEMO=$(readlink -f "$DEMO")
fi

mkdir -p "$OUTDIR"
export XLA_PYTHON_CLIENT_MEM_FRACTION=${MEM_FRACTION:-.60}
export MUJOCO_GL=egl

# 과제 2 는 위치가 성공 조건에 들어가므로 position 스케일을 키운다(env 기본과 동일).
# precision = exp(-4*ori_error). 기본 0.0 으로 꺼져 있었다.
# 진단(P0)에서 성공 밴드(err < 0.15) 안쪽이 보상 플래토임이 확인됐다:
# near_goal 1.0, success_band_bonus 2.0, coarse_align_bonus 4.0 이 전부 상수라
# orientation 보상 7.92 중 7.0(88%)이 밴드 안에서 평평하다. 정책이 여유를
# 만들 이유가 없어 밴드 가장자리에 표류하다가 넘어간다(+x 는 0.098/0.15).
# precision 은 밴드 안에서만 유효한 기울기를 준다(err 1.0 -> 0.018,
# 0.15 -> 0.549, 0.098 -> 0.677, 0.05 -> 0.819)라 도달 구간을 거의 건드리지 않는다.
PRECISION=${PRECISION:-0.0}
# 면별 샘플링 가중치(+z,-z,+x,-x,+y,-y). 빈 값이면 환경 기본값(균등)을 쓴다.
# 균등은 성공률이 51%인 면과 97%인 면에 같은 예산을 준다. "1,1,2.667,1,1,1" 이면
# +x 가 20% -> 40%.
FACE_WEIGHTS=${FACE_WEIGHTS:-}
# 규모 노브. 문헌(FastTD3 2505.22642, RSL-RL-SAC 2605.24975)의 절제 실험은
# "병렬 환경 수와 배치 크기"가 병렬 시뮬에서 off-policy 성능에 가장 크게
# 기여한다고 보고한다. 우리 기본값(512 / 1024)은 그들의 1/16 ~ 1/32 다.
# 표본 재사용률 = GRAD_UPDATES * BATCH / NUM_ENVS.
#   현재 32*1024/512 = 64배. RSL-RL-SAC 는 약 8배다(과학습 의심 구간).
NUM_ENVS=${NUM_ENVS:-512}
BATCH=${BATCH:-1024}
GRAD_UPDATES=${GRAD_UPDATES:-32}
# 초기 정책 표준편차. brax 기본 1.0. RSL-RL-SAC 는 이것을 실패 원인으로 지목하고
# 0.15 를 쓴다. 단 std 는 정책 파라미터라 LOAD_CKPT 로 이어받으면 덮어써지므로
# 처음부터 학습할 때만 의미가 있다.
INIT_NOISE_STD=${INIT_NOISE_STD:-}
# n-step 반환. 1 이면 brax 기본(1-step TD).
# 주의: 겹치지 않는 창이라 get_experience 한 번이 env 스텝을 N_STEP 배 소비한다.
# 같은 num_timesteps 에서 기울기 갱신 총량을 유지하려면 GRAD_UPDATES 를
# N_STEP 배로 올려야 한다(v21 E2 에서 갱신 총량을 절반으로 줄이자 명목이
# 82.8 -> 59.9 로 무너졌다. 40M 예산에서 제약은 데이터가 아니라 갱신 횟수다).
N_STEP=${N_STEP:-1}
# 에피소드 종료 시 환경을 완전히 재샘플링할지. 기본 false 면 각 환경이 자기 첫
# 에피소드의 목표 면·초기조건을 학습 내내 반복한다(실측 확인). 초기 상태
# 다양성이 num_envs 개로 묶이고 면별 배정이 시드에 고정된다.
FULL_RESET=${FULL_RESET:-false}
# residual RL: 학생 행동을 교사에 대한 교정량으로 해석한다.
#   a = clip(a_교사(s) + RESIDUAL_SCALE * a_학생(s), -1, 1)
# residual=0 이 이미 교사 성능이므로 DEMO(BC)는 쓰지 않는다.
RESIDUAL_TEACHER=${RESIDUAL_TEACHER:-}
# residual 학습에서 시연 혼합은 의미가 깨진다. 시연의 action 은 "절대 행동"인데
# residual 환경에서 같은 벡터는 "교사 대비 교정량"으로 해석되므로 critic 이
# Q(s, a) 를 잘못된 의미로 배운다. run_sac_exp.sh 와 같은 방어를 둔다.
if [ -n "$RESIDUAL_TEACHER" ] && [ -n "$DEMO" ]; then
  echo "[경고] residual 모드이므로 시연 혼합을 끈다 (요청값: $DEMO)" >&2
  DEMO=""
fi
RESIDUAL_SCALE=${RESIDUAL_SCALE:-0.3}
RESIDUAL_L2=${RESIDUAL_L2:-0.0}
# 목표 면 원-핫(6)을 관측에 붙인다. 관측 차원이 108->114 로 바뀌어 이전
# 체크포인트와 호환되지 않으므로 LOAD_CKPT 를 비워야 한다.
FACE_ONEHOT=${FACE_ONEHOT:-false}
# actor/critic 은닉층. 문헌(FastTD3, RSL-RL-SAC)은 critic 1024,512,256 을 쓴다.
# 바꾸면 파라미터 shape 이 달라져 LOAD_CKPT 와 호환되지 않는다.
HIDDEN=${HIDDEN:-512,256,128}
# joint_limit: 관절이 가동 범위 바깥 25%(SPiDR soft factor 0.75)에 들어간 정도.
# reward_config 오버라이드는 통째로 교체되므로 이 키가 빠지면 KeyError 가 난다.
# 0 이면 끔(기존 동작). 실기 기구 부담을 줄이려면 올린다.
# 한 스텝에 목표가 움직일 수 있는 최대 각도(rad). delta position 방식이라
# motor_targets = ctrl + action * ACTION_SCALE 이다.
# 0.2 는 스텝당 11.5도로, leaphand_관절각도.md 권장(0.05 = 2.9도)의 4배다.
# 문서는 "action-rate 가 sim2real 에서 joint-limit penalty 보다 중요"하다고
# 명시한다 — 목표가 빠르게 왕복하면 Dynamixel 에서 진동·과전류가 난다.
ACTION_SCALE=${ACTION_SCALE:-0.2}
JOINT_LIMIT_SCALE=${JOINT_LIMIT_SCALE:-0.0}
# 손 자세 유지 벌점. 상류(MuJoCo Playground)는 -0.5 인데 우리는 -0.001 로
# 500배 낮았고, orientation 을 5 -> 25 로 올린 것과 겹쳐 체감 비중이
# 1/2500 이 됐다. 그 결과 정책이 필요 없는 관절까지 한계로 꺾는다
# (가동범위 바깥 25% 체류율 46%).
HAND_POSE=${HAND_POSE:--0.001}
# 리셋 직후 큐브를 손 위에 안착시킬 스텝 수. 0 이면 42mm 낙하(상류 동작).
SETTLE_STEPS=${SETTLE_STEPS:-0}
# --- 연속 목표(연쇄) ---
CHAIN_GOALS=${CHAIN_GOALS:-false}
# 목표당 250~350스텝이라 800 이면 두 번째 목표를 거의 못 본다. 연쇄를 배우게
# 하려면 1600 이 필요하다(v51 에서 확인).
EPISODE_LENGTH=${EPISODE_LENGTH:-800}
# 학습에서는 에피소드 길이와 같게 두어 사실상 비활성으로 만든다. 600 이면
# +x 로 시작한 에피소드만 편향적으로 잘린다(+x 실측 최대 500스텝).
GOAL_TIMEOUT=${GOAL_TIMEOUT:-$EPISODE_LENGTH}
# 손가락 pip/dip 하한(과신전 금지). -9 면 끔. 팔 F 가 -0.2 로 학습됐다.
HYPEREXT_FLOOR=${HYPEREXT_FLOOR:--9.0}

SCALES="{\"orientation\": 25.0, \"progress\": 35.0, \"best_progress\": 80.0, \"direct_penalty\": 8.0, \"precision\": $PRECISION, \"fine_025\": 0.0, \"fine_020\": 0.0, \"position\": 4.0, \"termination\": -200.0, \"hand_pose\": $HAND_POSE, \"action_rate\": -0.005, \"joint_vel\": -0.001, \"energy\": -0.0002, \"joint_limit\": $JOINT_LIMIT_SCALE}"
FW_JSON=""
[ -n "$FACE_WEIGHTS" ] && FW_JSON=", \"face_weights\": [${FACE_WEIGHTS}]"
OVERRIDES="{\"impl\": \"jax\", \"success_threshold\": $THRESHOLD, \"position_threshold\": $POS_THRESHOLD, \"success_hold_steps\": $HOLD_STEPS, \"joint_limit_margin\": $JOINT_MARGIN, \"settle_steps\": $SETTLE_STEPS, \"chain_goals\": $CHAIN_GOALS, \"goal_timeout_steps\": $GOAL_TIMEOUT, \"hyperext_floor\": $HYPEREXT_FLOOR, \"actuator_force_limit\": $FORCE_LIMIT, \"hold_gated_success\": $GATED, \"hold_stairs\": $HOLD_STAIRS, \"reset_best_on_episode\": $RESET_BEST, \"action_scale\": $ACTION_SCALE, \"face_onehot_obs\": $FACE_ONEHOT, \"reward_config\": {\"scales\": $SCALES, \"success_reward\": $SUCCESS_REWARD, \"hold_bonus\": $HOLD_BONUS}$FW_JSON}"

ARGS=(
  --env_name LeapCubeFaceUp --impl jax
  --episode_length "$EPISODE_LENGTH"
  --num_envs "$NUM_ENVS" --num_eval_envs "$NUM_EVAL_ENVS" --num_evals "$EVALS"
  --batch_size "$BATCH" --grad_updates_per_step "$GRAD_UPDATES"
  --min_replay_size "$MINREPLAY" --max_replay_size "$MAX_REPLAY"
  --hidden_layer_sizes "$HIDDEN"
  --num_timesteps "$STEPS" --learning_rate "$LR"
  --discounting "$DISCOUNTING"
  --seed "$SEED"
  --obs_key state
  --playground_config_overrides "$OVERRIDES"
  --suffix "$SUFFIX"
  --logdir "$OUTDIR"
)
if [ -n "$DEMO" ]; then
  ARGS+=(--demo_data_path "$DEMO" --demo_ratio "$DEMO_RATIO" --bc_coef "$BC_COEF")
fi
if [ -n "$LOAD_CKPT" ]; then
  ARGS+=(--load_checkpoint_path "$LOAD_CKPT")
fi
if [ "$DOMAIN_RAND" = "true" ]; then
  ARGS+=(--domain_randomization)
fi
if [ -n "$RESIDUAL_TEACHER" ]; then
  ARGS+=(--residual_teacher "$RESIDUAL_TEACHER" --residual_scale "$RESIDUAL_SCALE" --residual_l2 "$RESIDUAL_L2")
fi
if [ "$FULL_RESET" = "true" ]; then
  ARGS+=(--full_reset)
fi
if [ "$N_STEP" -gt 1 ]; then
  ARGS+=(--n_step "$N_STEP")
fi
if [ -n "$INIT_NOISE_STD" ]; then
  ARGS+=(--init_noise_std "$INIT_NOISE_STD")
fi
if [ "$Q_LAYER_NORM" = "true" ]; then
  ARGS+=(--q_network_layer_norm)
elif [ "$Q_LAYER_NORM" = "false" ]; then
  ARGS+=(--noq_network_layer_norm)
fi
if [ "$SIMBA" = "true" ]; then
  ARGS+=(--simba --simba_hidden_dim "$SIMBA_HIDDEN")
fi

echo "=== $SUFFIX (과제2 SAC) ==="
echo "chain=$CHAIN_GOALS ep_len=$EPISODE_LENGTH timeout=$GOAL_TIMEOUT hyperext=$HYPEREXT_FLOOR threshold=$THRESHOLD pos=$POS_THRESHOLD demo=$DEMO ratio=$DEMO_RATIO bc=$BC_COEF steps=$STEPS seed=$SEED load=${LOAD_CKPT:-none} hand_pose=$HAND_POSE settle=$SETTLE_STEPS"
echo "margin=$JOINT_MARGIN force_limit=$FORCE_LIMIT hold_steps=$HOLD_STEPS gated=$GATED stairs=$HOLD_STAIRS reset_best=$RESET_BEST domain_rand=$DOMAIN_RAND simba=$SIMBA hold_bonus=$HOLD_BONUS success_reward=$SUCCESS_REWARD"
echo "q_layer_norm=${Q_LAYER_NORM:-설정기본값(True)} discounting=$DISCOUNTING envs=$NUM_ENVS batch=$BATCH grad_updates=$GRAD_UPDATES reuse=$(python3 -c "print(f'{$GRAD_UPDATES*$BATCH/$NUM_ENVS:.0f}')")x init_std=${INIT_NOISE_STD:-brax기본1.0} n_step=$N_STEP full_reset=$FULL_RESET residual=${RESIDUAL_TEACHER:+on(scale=$RESIDUAL_SCALE,l2=$RESIDUAL_L2)} face_onehot=$FACE_ONEHOT hidden=$HIDDEN precision=$PRECISION face_weights=${FACE_WEIGHTS:-균등}"
python learning/train_jax_sac.py "${ARGS[@]}"
