"""연속 목표(연쇄) 평가. 목표마다 성공/실패를 세고 연속 달성 횟수를 잰다.

왜 별도 스크립트인가
  `eval_face_up.py` 는 "에피소드당 목표 1개, 성공하면 종료" 전제로 짜여 있다.
  연쇄에서는 한 에피소드에 목표가 여러 개이고 성공이 종료가 아니다.

반드시 지킬 것 두 가지 (실측으로 데인 것들)

  1. **죽은 env 의 통계를 얼린다.** raw env 는 done 이 떠도 물리가 계속 돌기
     때문에, 마스킹하지 않으면 이미 끝난 env 가 계속 목표를 달성하며 카운터를
     올린다. 실측: 얼리지 않으면 연속 달성이 1.54 -> 1.98 로 부풀려졌다.

  2. **`max_hold` 를 배열로 남긴다.** 성공률은 `max_hold >= hold` 이므로 배열만
     있으면 임계값별 성공률을 사후에 전부 뽑을 수 있다. 유지 요구를 100 -> 20 으로
     바꿨기 때문에 기존 수치(hold=100)와 비교하려면 이것이 필요하다.

측정하는 것
  - 1번째 목표 성공률: 리셋 직후 시작이므로 기존 수치와 성격이 같다
  - k번째 목표 성공률: 정책이 만든 상태에서 시작 -> **재시작 가능성의 측정값**
  - 연속 달성 횟수: DeXtreme 식 대표 수치
  - 종료 사유 분포: 낙하 / 정체
"""

import functools
import json

import jax
import jax.numpy as jp
import numpy as np
from absl import app
from absl import flags
from brax.training import networks as brax_networks
from brax.training.acme import running_statistics
from brax.training.agents.ppo import checkpoint as ppo_checkpoint
from brax.training.agents.sac import checkpoint as sac_checkpoint
from brax.training.agents.sac import networks as sac_networks

from mujoco_playground import registry
from mujoco_playground.config import manipulation_params

_CHECKPOINT = flags.DEFINE_string("checkpoint", None, "평가할 체크포인트")
_ALGO = flags.DEFINE_enum("algo", "ppo", ["sac", "ppo"], "체크포인트 종류")
_DISTILLED = flags.DEFINE_string("distilled", None, "증류된 단일 망")
_NORM_FROM = flags.DEFINE_string("distill_norm_from", None, "증류용 정규화 템플릿")
_FIRST_FACE = flags.DEFINE_integer(
    "first_face", -1,
    "1번째 목표만 이 면으로 고정한다(-1 이면 무작위). 2번째부터는 항상 무작위이므로 "
    "면별 비교는 1번째 목표에 대해서만 성립한다.")
_STEPS = flags.DEFINE_integer("steps", 1500, "에피소드 최대 스텝")
_NUM_ENVS = flags.DEFINE_integer("num_envs", 256, "환경 수")
_HOLD = flags.DEFINE_integer("hold_steps", 20, "목표당 요구 유지 스텝")
_TIMEOUT = flags.DEFINE_integer("goal_timeout", 600, "목표 정체 판정 스텝")
_THRESHOLD = flags.DEFINE_float("success_threshold", 0.15, "면 각도 허용치(rad)")
_POS_THRESHOLD = flags.DEFINE_float("position_threshold", 0.015, "중심 허용 거리(m)")
_FORCE_LIMIT = flags.DEFINE_float("actuator_force_limit", 0.11, "토크 제한(Nm)")
_ACTION_SCALE = flags.DEFINE_float("action_scale", 0.2, "학습 때와 같아야 한다")
_SETTLE = flags.DEFINE_integer("settle_steps", 8, "리셋 후 큐브 안착 스텝")
# 손가락 pip/dip 하한(과신전 금지). 학습 때와 같아야 한다. -9 면 끔.
_HYPEREXT = flags.DEFINE_float("hyperext_floor", -9.0, "pip/dip 하한")
_ADELAY = flags.DEFINE_integer(
    "action_delay_steps", 0, "작동 지연 최대 스텝(에피소드마다 U{0..K})")
# 외력 섭동. 상류에 구현돼 있으나 기본이 꺼짐. 실기에서 손이 밀리거나
# 큐브가 외력을 받는 상황을 덮는다.
_PERT = flags.DEFINE_boolean("perturb", False, "외력 섭동 켜기")
# 연쇄 평가는 이 스크립트의 존재 이유이므로 기본이 켬이다. 플래그를 두는 것은
# 호출부가 --chain_goals 를 명시적으로 넘기는 것을 받아주기 위해서다(예전에는
# 없어서 FATAL 로 죽었다). --nochain_goals 로 단발 평가도 가능하다.
_CHAIN = flags.DEFINE_boolean("chain_goals", True, "성공 시 리셋 없이 다음 목표")
# residual 정책 평가.
#   학습 때 학생은 ResidualTeacherWrapper 아래에서 **교정량**을 낸다:
#       a = clip(a_교사(s) + scale * a_학생(s), -1, 1)
#   따라서 저장된 SAC 체크포인트를 단독으로 굴리면 교사가 빠져 의미가 없다.
#   실측(2026-08-30): v57 seed1 을 단독으로 굴리니 연쇄 0.00, 낙하 96.1%,
#   88스텝만에 사망했다. 같은 정책의 학습 보상은 교사 수준(23k)이었다.
#   -> 이 플래그를 주면 평가에서도 같은 방식으로 합성한다.
_RESID_TEACHER = flags.DEFINE_string(
    "residual_teacher", None, "residual 학생이면 교사 PPO 체크포인트 경로")
_RESID_SCALE = flags.DEFINE_float(
    "residual_scale", 0.1, "학습 때 쓴 residual_scale 과 같아야 한다")
_JOINT_MARGIN = flags.DEFINE_float("joint_limit_margin", 0.0, "구동 범위 축소")
_HIDDEN = flags.DEFINE_list("hidden_layer_sizes", ["512", "256", "128"], "은닉층")
_SEED = flags.DEFINE_integer("seed", 0, "난수 시드")
_OUT = flags.DEFINE_string("out", None, "결과 JSON")

# 임계값 격자. **설정한 hold 를 넘는 임계값은 구조적으로 0 이 나온다** -
# 연쇄는 success_hold_steps 가 config 의 hold 에 닿는 순간 목표를 바꾸고 0 으로
# 되돌리므로 max_hold 가 그 값을 넘을 수 없다. 실측(hold=20): hold50/hold100 이
# 전 목표에서 0.00%. hold=100 기준 비교가 필요하면 **hold_steps=100 으로 따로
# 한 번 더 돌려야 한다.** 격자는 설정값 이하로만 만든다.
def _hold_grid(hold):
  return tuple(sorted({max(1, int(hold * f)) for f in (0.25, 0.5, 0.75, 1.0)}))


def _make_env():
  return registry.load("LeapCubeFaceUp", config_overrides={
      "impl": "jax",
      "success_threshold": _THRESHOLD.value,
      "position_threshold": _POS_THRESHOLD.value,
      "success_hold_steps": _HOLD.value,
      "hold_gated_success": True,
      "actuator_force_limit": _FORCE_LIMIT.value,
      "action_scale": _ACTION_SCALE.value,
      "settle_steps": _SETTLE.value, "hyperext_floor": _HYPEREXT.value,
      "action_delay_steps": _ADELAY.value,
      "pert_config": {"enable": _PERT.value,
                      "linear_velocity_pert": [0.0, 3.0],
                      "angular_velocity_pert": [0.0, 0.5],
                      "pert_duration_steps": [1, 100],
                      "pert_wait_steps": [60, 150]},
      "joint_limit_margin": _JOINT_MARGIN.value,
      "chain_goals": _CHAIN.value,
      "goal_timeout_steps": _TIMEOUT.value,
      "target_face": _FIRST_FACE.value,
  })


def _load_policy(env):
  if _ALGO.value == "ppo" and not _DISTILLED.value:
    return ppo_checkpoint.load_policy(_CHECKPOINT.value, deterministic=True), True
  cfg = manipulation_params.brax_sac_config("LeapCubeReorient")
  kwargs = dict(cfg.network_factory)
  kwargs["hidden_layer_sizes"] = tuple(int(x) for x in _HIDDEN.value)
  net = functools.partial(sac_networks.make_sac_networks, **kwargs)(
      observation_size=env.observation_size["state"][0],
      action_size=env.action_size,
      preprocess_observations_fn=running_statistics.normalize)
  if _DISTILLED.value:
    from etils import epath
    import flax
    tmpl = {"norm": sac_checkpoint.load(_NORM_FROM.value)[0],
            "policy": net.policy_network.init(jax.random.PRNGKey(0))}
    r = flax.serialization.from_bytes(
        tmpl, epath.Path(_DISTILLED.value).resolve().read_bytes())
    params = (r["norm"], r["policy"])
  else:
    params = sac_checkpoint.load(_CHECKPOINT.value)
  return jax.jit(sac_networks.make_inference_fn(net)(
      params, deterministic=True)), False


def main(argv):
  del argv
  brax_networks.KERNEL_INITIALIZER.setdefault(None, None)
  env = _make_env()
  policy, is_ppo = _load_policy(env)
  if _RESID_TEACHER.value:
    if is_ppo or _DISTILLED.value:
      raise ValueError(
          "--residual_teacher 는 SAC 학생 체크포인트에만 쓴다. 증류망은 이미 "
          "합성된 단일망이므로 교사를 또 더하면 안 된다.")
    _tfn = ppo_checkpoint.load_policy(_RESID_TEACHER.value, deterministic=True)
    _student, _sc = policy, _RESID_SCALE.value

    def _composed(obs_dict, k):
      a_t, _ = _tfn(obs_dict, k)
      a_r, _ = _student(obs_dict["state"], k)
      return jp.clip(a_t + _sc * a_r, -1.0, 1.0), {}

    policy, is_ppo = jax.jit(_composed), True   # dict 관측을 받는다
    print(f"residual 합성: 교사={_RESID_TEACHER.value} scale={_sc}")
  n = _NUM_ENVS.value
  reset = jax.jit(jax.vmap(env.reset))
  step = jax.jit(jax.vmap(env.step))

  st = reset(jax.random.split(jax.random.PRNGKey(_SEED.value), n))
  key = jax.random.PRNGKey(_SEED.value + 1)

  alive = np.ones(n)
  # 목표별 최대 유지 스텝. 목표 인덱스는 goal_count 로 안다.
  # 넉넉히 잡고 실제로 쓰인 만큼만 집계한다.
  max_goals = 16
  hold_per_goal = np.zeros((n, max_goals))
  frozen_goals = np.zeros(n)          # 죽는 순간의 goal_count
  died_at = np.full(n, float(_STEPS.value))
  dropped = np.zeros(n)
  stuck = np.zeros(n)

  prev_gc = np.zeros(n, dtype=int)
  for t in range(1, _STEPS.value + 1):
    key, ak = jax.random.split(key)
    obs = st.obs if is_ppo else st.obs["state"]
    a, _ = policy(obs, ak)
    st = step(st, a)

    live = alive > 0.5
    gc = np.asarray(st.metrics["goal_count"]).astype(int)
    hold = np.asarray(st.metrics["success_hold_steps"])
    # **인덱스는 이번 스텝 이전의 goal_count 를 쓴다.** 목표를 달성한 스텝에서는
    # metrics 의 success_hold_steps 가 (교체 전) 20 인데 goal_count 는 이미 1 로
    # 올라가 있다(step 안에서 metrics 기록 순서가 다르다: hold 는 452행,
    # goal_count 는 _chain_next_goal 뒤). gc 로 색인하면 목표 0 의 유지 기록이
    # 목표 1 칸에 들어가 한 칸씩 밀린다.
    idx = np.clip(prev_gc, 0, max_goals - 1)
    cur = hold_per_goal[np.arange(n), idx]
    hold_per_goal[np.arange(n), idx] = np.where(
        live, np.maximum(cur, hold), cur)
    prev_gc = gc

    d = np.asarray(st.done)
    newly = live & (d > 0.5)
    frozen_goals = np.where(live, gc, frozen_goals)
    died_at = np.where(newly, float(t), died_at)
    dropped = np.where(newly, np.asarray(st.metrics["term_dropped"]), dropped)
    stuck = np.where(newly, np.asarray(st.metrics["term_stuck"]), stuck)
    alive = alive * (1.0 - d)
    if alive.sum() == 0:
      break

  # --- 집계 ---
  # k번째 목표를 "시도했다" = 그 env 가 goal_count >= k 에 도달했거나
  # 죽기 전에 그 목표를 진행 중이었다.
  grid = _hold_grid(_HOLD.value)
  per_goal = []
  for k in range(max_goals):
    attempted = frozen_goals >= k          # k번째 목표를 시도한 env
    if attempted.sum() < max(4, 0.02 * n):
      break
    h = hold_per_goal[attempted, k]
    row = {"goal_index": k + 1, "attempts": int(attempted.sum())}
    for thr in grid:
      row[f"success_hold{thr}"] = float((h >= thr).mean())
    row["max_hold_mean"] = float(h.mean())
    row["max_hold_p50"] = float(np.median(h))
    per_goal.append(row)

  out = {
      "consecutive_goals_mean": float(frozen_goals.mean()),
      "consecutive_goals_median": float(np.median(frozen_goals)),
      "consecutive_goals_max": float(frozen_goals.max()),
      "consecutive_hist": np.bincount(
          frozen_goals.astype(int), minlength=10)[:10].tolist(),
      "episode_len_mean": float(died_at.mean()),
      "survived_frac": float((died_at >= _STEPS.value).mean()),
      "term_dropped_frac": float(dropped.mean()),
      "term_stuck_frac": float(stuck.mean()),
      "per_goal": per_goal,
      "config": {"hold_steps": _HOLD.value, "goal_timeout": _TIMEOUT.value,
                 "steps": _STEPS.value, "first_face": _FIRST_FACE.value,
                 "num_envs": n, "settle_steps": _SETTLE.value, "hyperext_floor": _HYPEREXT.value,
      "action_delay_steps": _ADELAY.value,
      "pert_config": {"enable": _PERT.value,
                      "linear_velocity_pert": [0.0, 3.0],
                      "angular_velocity_pert": [0.0, 0.5],
                      "pert_duration_steps": [1, 100],
                      "pert_wait_steps": [60, 150]},
                 "action_scale": _ACTION_SCALE.value,
                 "joint_limit_margin": _JOINT_MARGIN.value},
  }

  print(f"연속 달성  평균 {out['consecutive_goals_mean']:.2f}  "
        f"중앙값 {out['consecutive_goals_median']:.0f}  "
        f"최대 {out['consecutive_goals_max']:.0f}")
  print(f"분포(0~9)  {out['consecutive_hist']}")
  print(f"에피소드   평균 {out['episode_len_mean']:.0f}스텝  "
        f"끝까지 생존 {out['survived_frac']*100:.1f}%")
  print(f"종료 사유  낙하 {out['term_dropped_frac']*100:.1f}%  "
        f"정체 {out['term_stuck_frac']*100:.1f}%")
  print(f"\n{'목표':>4}{'시도':>7}" + "".join(f"{'hold'+str(h):>10}" for h in grid)
        + f"{'maxHold평균':>12}")
  for r in per_goal:
    print(f"{r['goal_index']:>4}{r['attempts']:>7}"
          + "".join(f"{r['success_hold'+str(h)]*100:>10.2f}" for h in grid)
          + f"{r['max_hold_mean']:>12.1f}")

  if _OUT.value:
    with open(_OUT.value, "w", encoding="utf-8") as fp:
      json.dump(out, fp, indent=2, ensure_ascii=False)
    print(f"\n저장: {_OUT.value}")


if __name__ == "__main__":
  app.run(main)
