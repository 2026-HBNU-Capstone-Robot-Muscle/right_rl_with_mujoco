#!/usr/bin/env bash
# residual SAC + DAgger 증류. 과제 2(LeapCubeFaceUp) 배포용 단일망을 만든다.
#
# 구조
#   [1] residual SAC : a = clip(a_교사(s) + 0.1*a_학생(s), -1, 1)
#       핸드오프 진단에서 학생이 **자기 궤적 위에서만** 못 한다는 복합 오차를
#       확인했고, residual 이 그것을 구조로 없앤다(ResFiT arXiv:2509.19301).
#       residual 에서는 시연 혼합을 쓰지 않는다 - 시연의 action 은 절대 행동인데
#       residual 환경에서는 교정량으로 해석되어 critic 이 Q(s,a) 를 잘못 배운다.
#   [2] 합성 평가    : **교사를 더해서** 평가해야 한다.
#       저장된 SAC 체크포인트를 단독으로 굴리면 0.1배 교정량이 절대 명령이 되어
#       손이 멈춘다(실측: 연쇄 0.00, 낙하 96%). 평가가 전부 0 이 되면 최고점
#       선택이 사실상 무작위가 된다.
#   [3] DAgger 증류  : 교사+residual 결합 정책을 단일망으로.
#       토크 4구간에서 데이터를 모은다. 한 토크에서만 증류하면 그 조건의 입출력
#       관계만 배워 다른 토크에서 무너진다.
#
# 연쇄 설정을 학습·평가·증류에 **전부 같이** 넘겨야 한다.
#   chain_goals, episode_length, hold, settle, hyperext 중 하나라도 어긋나면
#   교사와 학생이 다른 과제를 푼다.
set -uo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PG=${PG:?PG 를 mujoco_playground 저장소 루트로 지정하세요 (learning/ 이 있는 곳)}
ROOT=${ROOT:-$HERE}
OUT=${OUT:-$HERE/out}
TEACHER=${TEACHER:?교사 PPO 체크포인트 디렉터리를 지정하세요}
export PG ROOT
SCALE=${SCALE:-0.1}
STEPS=${STEPS:-40000000}
SEEDS=${SEEDS:-"1 2"}
FORCE=${FORCE:-0.11}
cd "$HERE"; source ${VENV:-~/rl}/bin/activate; mkdir -p "$OUT"

COMMON="--chain_goals --goal_timeout 600 --settle_steps 8 --hyperext_floor -0.2 \
        --hold_steps 20 --action_scale 0.2 --actuator_force_limit $FORCE"
RES="--residual_teacher $TEACHER --residual_scale $SCALE"

echo "===== [1] residual SAC ====="
for sd in $SEEDS; do
  d=$OUT/sac_s$sd
  [ "$(ls -d $d/*/checkpoints/0* 2>/dev/null | wc -l)" -ge 3 ] && { echo "  있음 s$sd"; continue; }
  NUM_ENVS=2048 BATCH=4096 GRAD_UPDATES=32 SEED=$sd MEM_FRACTION=.70 \
  FULL_RESET=true RESIDUAL_TEACHER="$TEACHER" RESIDUAL_SCALE=$SCALE INIT_NOISE_STD=0.15 \
  SUFFIX=res_s$sd OUTDIR=$d DEMO="" LOAD_CKPT="" \
    CHAIN_GOALS=true EPISODE_LENGTH=1600 HYPEREXT_FLOOR=-0.2 SETTLE_STEPS=8 \
    HOLD_STAIRS=1 SUCCESS_REWARD=2000.0 DOMAIN_RAND=true GATED=true \
    STEPS=$STEPS EVALS=9 LR=3e-5 THRESHOLD=0.15 POS_THRESHOLD=0.015 \
    FORCE_LIMIT=$FORCE HOLD_STEPS=20 \
    "$HERE/run_faceup_sac.sh" > "$OUT/sac_s$sd.log" 2>&1 || { tail -25 "$OUT/sac_s$sd.log"; exit 1; }
done

echo "===== [2] 합성 평가 (끝 3개 x 시드) ====="
rm -f "$OUT"/eval_*.json "$OUT"/eval_*.ckpt
for sd in $SEEDS; do
  i=0
  for D in $(ls -d $OUT/sac_s$sd/*/checkpoints/0* | sort | tail -3); do
    i=$((i+1))
    (cd "$PG" && export MUJOCO_GL=egl XLA_PYTHON_CLIENT_MEM_FRACTION=.6
     python learning/eval_chain.py --checkpoint "$D" --algo sac $RES \
       --num_envs 256 --steps 1500 $COMMON --out "$OUT/eval_s${sd}_t${i}.json") 2>&1 | head -4
    echo "$D" > "$OUT/eval_s${sd}_t${i}.ckpt"
  done
done
BEST=$(OUT=$OUT python3 - <<'PY'
import glob, json, os, sys
best, bs = None, -1.0
for f in sorted(glob.glob(os.environ["OUT"] + "/eval_s*_t*.json")):
    ck = f[:-5] + ".ckpt"
    if not os.path.exists(ck): continue
    s = json.load(open(f))["consecutive_goals_mean"]
    print(f"  {os.path.basename(f):<22} 연쇄 {s:.2f}", file=sys.stderr)
    if s > bs: bs, best = s, open(ck).read().strip()
print(f"  선택 연쇄 {bs:.2f}", file=sys.stderr); print(best)
PY
)
[ -z "$BEST" ] && { echo "선택 실패" >&2; exit 1; }

echo "===== [3] DAgger 증류 ====="
[ -e "$OUT/distilled/policy" ] || (cd "$PG" && export MUJOCO_GL=egl XLA_PYTHON_CLIENT_MEM_FRACTION=.60
  python learning/distill_residual.py --env_name LeapCubeFaceUp \
    --sac_checkpoint "$BEST" --ppo_checkpoint "$TEACHER" --residual_scale $SCALE \
    --action_scale 0.2 --torque_range 0.09,0.11,0.16,0.21 \
    --settle_steps 8 --chain_goals --goal_timeout 600 --hyperext_floor -0.2 \
    --rounds 4 --rollout_steps 200 --num_envs 384 --epochs 25 \
    --success_threshold 0.15 --hold_steps 20 --out "$OUT/distilled/policy") || exit 1

echo "===== [4] 증류 평가 (토크 3구간) + 교사 대조 ====="
for FL in 0.11 0.16 0.21; do
  (cd "$PG" && export MUJOCO_GL=egl XLA_PYTHON_CLIENT_MEM_FRACTION=.6
   echo "--- 증류 @ $FL"
   python learning/eval_chain.py --distilled "$OUT/distilled/policy" \
     --distill_norm_from "$BEST" --algo sac --num_envs 256 --steps 1500 \
     --chain_goals --goal_timeout 600 --settle_steps 8 --hyperext_floor -0.2 \
     --hold_steps 20 --action_scale 0.2 --actuator_force_limit $FL \
     --out "$OUT/distill_f${FL}.json"
   echo "--- 교사 @ $FL"
   python learning/eval_chain.py --checkpoint "$TEACHER" --algo ppo \
     --num_envs 256 --steps 1500 --chain_goals --goal_timeout 600 \
     --settle_steps 8 --hyperext_floor -0.2 --hold_steps 20 --action_scale 0.2 \
     --actuator_force_limit $FL --out "$OUT/teacher_f${FL}.json") 2>&1 | head -12
done

echo "===== [5] 배포용 내보내기 ====="
(cd "$HERE/../deploy" && python export_policy.py \
   --distilled "$OUT/distilled/policy" --norm_from "$BEST" \
   --out "$HERE/../policy/policy_np.npz")
cp "$OUT/distilled/policy" "$HERE/../policy/distilled_policy"
echo "완료. Leap_Test/policy/policy_np.npz 가 배포 산출물이다."
