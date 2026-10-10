"""결합 정책(교사+residual)을 교사 없이 도는 단일 망으로 증류한다.

왜 필요한가
  residual 정책은 배포 시 교사(PPO)와 학생(SAC) 두 망을 함께 돌려야 한다.
  실기 검증의 전제조건이 배포 가능성이므로 하나로 합친다.

왜 순수 BC 가 아니라 DAgger 인가
  핸드오프 진단(2026-08-27)에서 확인했다: 교사 궤적 위에서 학습한 정책은
  자기 궤적 위에서 5~16배 벌어진다(복합 오차). 결합 정책의 궤적만 모아
  지도학습하면 같은 실패를 반복한다.

  DAgger 는 **학생이 굴린 상태**에서 결합 정책(전문가)의 라벨을 받는다.
  우리 경우 전문가가 결정적 함수라 언제든 무료로 질의할 수 있어, DAgger 의
  통상적 비용(사람 라벨링)이 0 이다.

절차
  라운드 r 마다
    1) 현재 학생 π_r 로 롤아웃한다 (r=0 이면 전문가로 굴린다)
    2) 방문한 상태마다 전문가 라벨 a* = clip(a_교사 + scale·a_res) 를 기록한다
    3) 누적 데이터셋에 넣고 지도학습으로 π_{r+1} 을 갱신한다
  DAgger 의 요지는 데이터셋을 **버리지 않고 누적**하는 것이다. 교체하면
  이전 라운드의 커버리지를 잃는다(v9 가 그렇게 실패했다).

주의
  학생은 교사와 같은 관측(state, 108차원)을 받고 같은 행동 공간을 쓴다.
  즉 배포 시 SAC 정책 하나만 있으면 된다.
"""

import functools
import json
import time

from absl import app
from absl import flags
from brax.training.acme import running_statistics
from brax.training.agents.sac import checkpoint as sac_checkpoint
from brax.training.agents.sac import networks as sac_networks
from brax.training import networks as brax_networks
from brax.training.agents.ppo import checkpoint as ppo_checkpoint
from etils import epath
import flax
import jax
import jax.numpy as jp
import numpy as np
import optax
import orbax.checkpoint as ocp

from mujoco_playground import registry
from mujoco_playground.config import manipulation_params
import residual_wrapper

_ENV = flags.DEFINE_string("env_name", "LeapCubeFaceUp", "환경")
_SAC = flags.DEFINE_string("sac_checkpoint", None, "residual 학생", required=True)
_PPO = flags.DEFINE_string("ppo_checkpoint", None, "base 교사", required=True)
_SCALE = flags.DEFINE_float("residual_scale", 0.1, "교정량 크기")
_ROUNDS = flags.DEFINE_integer("rounds", 5, "DAgger 라운드 수")
_ROLLOUT = flags.DEFINE_integer("rollout_steps", 400, "라운드당 롤아웃 스텝")
_NUM_ENVS = flags.DEFINE_integer("num_envs", 512, "환경 수")
_EPOCHS = flags.DEFINE_integer("epochs", 40, "라운드당 지도학습 에폭")
_BATCH = flags.DEFINE_integer("batch_size", 4096, "지도학습 배치")
_LR = flags.DEFINE_float("learning_rate", 3e-4, "지도학습 학습률")
_HIDDEN = flags.DEFINE_list("hidden_layer_sizes", ["512", "256", "128"], "은닉층")
_FORCE_LIMIT = flags.DEFINE_float("actuator_force_limit", 0.11, "토크 제한")
_ACTION_SCALE = flags.DEFINE_float("action_scale", 0.2, "residual 학생 학습 때와 같아야 한다")
_SETTLE = flags.DEFINE_integer("settle_steps", 0, "리셋 후 큐브 안착 스텝")
# 연쇄 설정은 **학습과 반드시 같아야 한다.** 다르면 교사·학생이 방문하는 상태
# 분포가 달라져 DAgger 의 전제(학생 궤적에서 전문가 라벨을 받는다)가 깨진다.
_CHAIN = flags.DEFINE_boolean("chain_goals", False, "성공 시 새 목표로 이어간다")
_GOAL_TIMEOUT = flags.DEFINE_integer("goal_timeout", 600, "목표 정체 판정 스텝")
_HYPEREXT = flags.DEFINE_float("hyperext_floor", -9.0, "pip/dip 하한(과신전 금지)")
_ADELAY = flags.DEFINE_integer(
    "action_delay_steps", 0, "작동 지연 최대 스텝(에피소드마다 U{0..K})")
_HOLD_STEPS = flags.DEFINE_integer("hold_steps", 100, "요구 유지 스텝")
_THRESHOLD = flags.DEFINE_float("success_threshold", 0.15, "성공 임계값")
_FACES = flags.DEFINE_list("faces", ["1", "2", "3", "4", "5"], "과제2 목표 면")
_TORQUE_RANGE = flags.DEFINE_list(
    "torque_range", None,
    "지정하면 이 토크값들을 번갈아 쓰며 데이터를 모은다 (예: 0.09,0.11,0.16,0.21). "
    "증류를 한 토크에서만 하면 그 조건의 입출력 관계만 배워 다른 토크에서 "
    "무너진다(실측: 0.11 증류 -> 0.21 에서 99.3 -> 88.2). 결합 정책은 교사가 "
    "실시간 반응하므로 버티지만 증류 정책은 못 버틴다.")
_OUT = flags.DEFINE_string("out", None, "증류된 정책 저장 경로", required=True)


def _make_env(face_idx=None, force_limit=None):
  ov = {
      "impl": "jax",
      "success_threshold": _THRESHOLD.value,
      "success_hold_steps": _HOLD_STEPS.value,
      "actuator_force_limit": _FORCE_LIMIT.value if force_limit is None
                              else float(force_limit),
      "action_scale": _ACTION_SCALE.value,
      "settle_steps": _SETTLE.value,
      "chain_goals": _CHAIN.value,
      "goal_timeout_steps": _GOAL_TIMEOUT.value,
      "hyperext_floor": _HYPEREXT.value,
      "action_delay_steps": _ADELAY.value,
  }
  if _ENV.value == "LeapCubeFaceUp":
    ov["position_threshold"] = 0.015
    if face_idx is not None:
      ov["target_face"] = face_idx
  else:
    ov["goal_angle_scale"] = 1.0
    ov["goal_angle_scale_min"] = 0.375
  return registry.load(_ENV.value, config_overrides=ov)


def main(argv):
  del argv
  brax_networks.KERNEL_INITIALIZER.setdefault(None, None)
  teacher_fn = ppo_checkpoint.load_policy(_PPO.value, deterministic=True)
  sac_params = sac_checkpoint.load(_SAC.value)

  probe = _make_env()
  obs_size = probe.observation_size["state"][0]
  act_size = probe.action_size
  cfg = manipulation_params.brax_sac_config("LeapCubeReorient")
  kwargs = dict(cfg.network_factory)
  kwargs["hidden_layer_sizes"] = tuple(int(x) for x in _HIDDEN.value)
  net = functools.partial(sac_networks.make_sac_networks, **kwargs)(
      observation_size=obs_size, action_size=act_size,
      preprocess_observations_fn=running_statistics.normalize)
  res_fn = jax.jit(
      sac_networks.make_inference_fn(net)(sac_params, deterministic=True))

  # 증류 대상: 같은 구조의 새 정책망. 배포 시 이것 하나만 쓴다.
  key = jax.random.PRNGKey(0)
  key, ik = jax.random.split(key)
  student_params = net.policy_network.init(ik)
  # 정규화 통계는 residual 학생 것을 그대로 물려받는다(같은 관측 분포).
  norm_params = sac_params[0]

  def student_mean(params, obs):
    """결정적 행동 = tanh(정책 평균)."""
    logits = net.policy_network.apply(norm_params, params, obs)
    return jp.tanh(logits[..., :act_size])

  def expert_action(state):
    """전문가 = clip(a_교사 + scale·a_residual)."""
    a_t, _ = teacher_fn(state.obs, key)
    a_r, _ = res_fn(state.obs["state"], key)
    return jp.clip(a_t + _SCALE.value * a_r, -1.0, 1.0)

  opt = optax.adam(_LR.value)
  opt_state = opt.init(student_params)

  @jax.jit
  def train_step(params, opt_state, obs, act):
    def loss_fn(p):
      return jp.mean((student_mean(p, obs) - act) ** 2)
    loss, grads = jax.value_and_grad(loss_fn)(params)
    updates, opt_state = opt.update(grads, opt_state)
    return optax.apply_updates(params, updates), opt_state, loss

  faces = [int(f) for f in _FACES.value] if _ENV.value == "LeapCubeFaceUp" else [None]
  torques = ([float(x) for x in _TORQUE_RANGE.value] if _TORQUE_RANGE.value
             else [_FORCE_LIMIT.value])
  # (면, 토크) 조합마다 환경을 하나씩. jit 캐시가 조합 수만큼 생긴다.
  envs = {(f, q): _make_env(f, q) for f in faces for q in torques}
  steps = {k: jax.jit(jax.vmap(v.step)) for k, v in envs.items()}
  resets = {k: jax.jit(jax.vmap(v.reset)) for k, v in envs.items()}
  print(f"수집 조건: 면 {faces} × 토크 {torques}")

  X, Y = [], []
  for r in range(_ROUNDS.value):
    t0 = time.time()
    n_new = 0
    for (f, q) in envs:
      st = resets[(f, q)](
          jax.random.split(jax.random.PRNGKey(100 + r), _NUM_ENVS.value))
      for t in range(_ROLLOUT.value):
        a_star = expert_action(st)
        X.append(np.asarray(st.obs["state"]))
        Y.append(np.asarray(a_star))
        n_new += a_star.shape[0]
        # r=0 은 전문가로 굴려 시작점을 잡고, 이후는 **학생**으로 굴린다.
        # 이것이 DAgger 의 요체다: 학생이 실제로 방문하는 상태에서 라벨을 받는다.
        drive = a_star if r == 0 else student_mean(student_params, st.obs["state"])
        st = steps[(f, q)](st, drive)
    Xa = np.concatenate(X); Ya = np.concatenate(Y)
    # 지도학습 (누적 데이터셋 전체)
    rng = np.random.default_rng(r)
    for e in range(_EPOCHS.value):
      idx = rng.permutation(len(Xa))
      tot = 0.0
      for i in range(0, len(idx) - _BATCH.value + 1, _BATCH.value):
        b = idx[i:i + _BATCH.value]
        student_params, opt_state, l = train_step(
            student_params, opt_state, jp.asarray(Xa[b]), jp.asarray(Ya[b]))
        tot += float(l)
      last = tot / max(1, len(idx) // _BATCH.value)
    print(f"라운드 {r}: 신규 {n_new:,} 누적 {len(Xa):,}  MSE {last:.5f}  "
          f"({time.time()-t0:.0f}초)", flush=True)

  # orbax 로 저장하면 sharding 정보가 붙어 다른 장치에서 복원이 깨진다
  # (GPU 저장 -> CPU 복원 시 "sharding ... Got None"). flax 직렬화로 남긴다.
  out = epath.Path(_OUT.value)
  out.parent.mkdir(parents=True, exist_ok=True)
  blob = flax.serialization.to_bytes(
      jax.device_get({"norm": norm_params, "policy": student_params}))
  out.write_bytes(blob)
  print(f"저장: {out} ({len(blob)/1e6:.1f}MB)")


if __name__ == "__main__":
  app.run(main)
