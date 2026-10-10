"""체크포인트를 고정 목표각에서 굴려 데모 영상을 만든다.

학습 스크립트가 자동 저장하는 rollout0.mp4 는 데모용으로 부적합하다.
  - 학습 최종 체크포인트를 쓴다 (고정각 평가로 고른 최고 체크포인트가 아님)
  - 목표각이 무작위라 어떤 난이도인지 알 수 없다
  - episode_length(800스텝 = 40초) 를 통째로 찍는데 정책은 40~60스텝에 성공
    종료하므로 영상의 90% 가 종료 후 잔여 구간이다

여기서는 각도별로 여러 에피소드를 돌리고 **각 에피소드가 끝나는 시점에서 잘라**
이어 붙인다. 성공/실패와 소요 스텝을 함께 출력해 영상과 대조할 수 있게 한다.
"""

import functools

from absl import app
from absl import flags
from brax.training.acme import running_statistics
from brax.training.agents.sac import checkpoint as sac_checkpoint
from brax.training.agents.sac import networks as sac_networks
from brax.training.agents.ppo import checkpoint as ppo_checkpoint
from brax.training import networks as brax_networks
from etils import epath
import jax
import jax.numpy as jp
import mediapy as media
import mujoco
import numpy as np

from mujoco_playground import registry
from mujoco_playground.config import manipulation_params


_CHECKPOINT = flags.DEFINE_string("checkpoint", None, "렌더링할 체크포인트", required=True)
_ALGO = flags.DEFINE_enum("algo", "sac", ["sac", "ppo"], "체크포인트 종류")
_OUT_DIR = flags.DEFINE_string("out_dir", None, "mp4 저장 폴더", required=True)
_ENV_NAME = flags.DEFINE_string(
    "env_name", "LeapCubeReorient", "LeapCubeReorient(과제1) 또는 LeapCubeFaceUp(과제2)"
)
_ANGLES = flags.DEFINE_list(
    "goal_angle_scales", ["0.5", "0.75", "1.0"], "과제1: 목표각 목록"
)
_FACES = flags.DEFINE_list("faces", ["1", "2", "3", "4", "5"], "과제2: 목표 면 목록")
_POSITION_THRESHOLD = flags.DEFINE_float(
    "position_threshold", 0.015, "과제2: 손바닥 중심 허용 거리(m)"
)
_FACE_NAMES = {0: "posz", 1: "negz", 2: "posx", 3: "negx", 4: "posy", 5: "negy"}
_EPISODES = flags.DEFINE_integer("episodes", 4, "각도당 에피소드 수")
_MAX_STEPS = flags.DEFINE_integer("max_steps", 200, "에피소드당 최대 스텝")
_SUCCESS_THRESHOLD = flags.DEFINE_float("success_threshold", 0.8, "성공 임계값(rad)")
_HIDDEN = flags.DEFINE_list("hidden_layer_sizes", ["512", "256", "128"], "은닉층 크기")
_SEED = flags.DEFINE_integer("seed", 0, "난수 시드")
_FORCE_LIMIT = flags.DEFINE_float(
    "actuator_force_limit", 0.11,
    "모터 토크 제한(Nm). 0.11 이 실기 기본(300mA) = 명목 조건이다.")
_HOLD_STEPS = flags.DEFINE_integer(
    "hold_steps", 100, "요구 유지 스텝. 평가와 같은 100(5초)을 기본으로 둔다.")
_DISTILLED = flags.DEFINE_string(
    "distilled", None,
    "distill_residual.py 가 저장한 단일 망(교사 불필요). 지정 시 --checkpoint 는 "
    "정규화 통계 템플릿으로만 쓰인다.")
_ACTION_SCALE = flags.DEFINE_float("action_scale", 0.2, "학습 때와 같아야 한다")
# 리셋 직후 큐브를 손 위에 안착시킬 스텝 수. 학습 때와 같아야 한다.
_SETTLE = flags.DEFINE_integer("settle_steps", 0, "리셋 후 큐브 안착 스텝")
# 손가락 pip/dip 하한(과신전 금지). 학습 때와 같아야 한다. -9 면 끔.
_HYPEREXT = flags.DEFINE_float("hyperext_floor", -9.0, "pip/dip 하한")
_ADELAY = flags.DEFINE_integer(
    "action_delay_steps", 0, "작동 지연 최대 스텝(에피소드마다 U{0..K})")
# 연속 목표. 학습이 연쇄였으면 여기서도 켜야 같은 분포를 본다.
# 끄면 첫 목표에서 에피소드가 끝나므로 "완수 뒤 이어가는" 구간을 못 본다.
_CHAIN = flags.DEFINE_boolean("chain_goals", False, "성공 시 새 목표로 이어간다")
_GOAL_TIMEOUT = flags.DEFINE_integer("goal_timeout", 600, "목표 정체 판정 스텝")
_FPS = flags.DEFINE_float("fps", 30.0, "출력 FPS")
_WIDTH = flags.DEFINE_integer("width", 640, "영상 가로")
_HEIGHT = flags.DEFINE_integer("height", 480, "영상 세로")
_TAIL_FRAMES = flags.DEFINE_integer(
    "tail_frames", 10, "성공 종료 후 결과를 보여주기 위해 덧붙일 정지 프레임 수"
)

_SCALES = {
    "orientation": 25.0, "progress": 35.0, "best_progress": 80.0,
    "direct_penalty": 8.0, "precision": 0.0, "fine_025": 0.0, "fine_020": 0.0,
    "position": 0.5, "termination": -200.0, "hand_pose": -0.001,
    "action_rate": -0.005, "joint_vel": -0.001, "energy": -2e-4,
    # joint_limit: 관절이 가동 범위 바깥 25%(SPiDR soft factor 0.75)에
    # 들어간 정도. reward_config 오버라이드는 통째로 교체되므로 이 키가
    # 빠지면 KeyError 가 난다. 평가는 보상과 무관하므로 0.0 으로 둔다.
    "joint_limit": 0.0,
}


def _is_faceup():
  return _ENV_NAME.value == "LeapCubeFaceUp"


def _make_env(cond):
  """cond 는 과제1이면 goal_angle_scale, 과제2면 목표 면 인덱스."""
  if _is_faceup():
    # 과제2 설정에는 goal_angle_* 가 없다. reward_config 는 env 기본값을 쓴다.
    return registry.load("LeapCubeFaceUp", config_overrides={
        "impl": "jax",
        "success_threshold": _SUCCESS_THRESHOLD.value,
        "position_threshold": _POSITION_THRESHOLD.value,
        "success_hold_steps": _HOLD_STEPS.value,
        "actuator_force_limit": _FORCE_LIMIT.value,
        "target_face": int(cond),
        "action_scale": _ACTION_SCALE.value, "settle_steps": _SETTLE.value, "hyperext_floor": _HYPEREXT.value,
      "action_delay_steps": _ADELAY.value, "chain_goals": _CHAIN.value,
          "goal_timeout_steps": _GOAL_TIMEOUT.value,
    })
  return registry.load("LeapCubeReorient", config_overrides={
      "impl": "jax",
      "success_threshold": _SUCCESS_THRESHOLD.value,
      "success_hold_steps": _HOLD_STEPS.value,
      "actuator_force_limit": _FORCE_LIMIT.value,
      "goal_angle_scale": float(cond),
      "goal_angle_scale_min": 0.0,
      "action_scale": _ACTION_SCALE.value, "settle_steps": _SETTLE.value, "hyperext_floor": _HYPEREXT.value,
      "action_delay_steps": _ADELAY.value, "chain_goals": _CHAIN.value,
          "goal_timeout_steps": _GOAL_TIMEOUT.value,
      "reward_config": {
          "scales": _SCALES, "success_reward": 75.0, "hold_bonus": 0.0,
      },
  })


def main(argv):
  del argv
  # PPO 체크포인트 설정에 init_fn 이름이 None 으로 남아 있어 조회가 깨진다.
  # 다른 평가 스크립트와 같은 방어를 둔다.
  brax_networks.KERNEL_INITIALIZER.setdefault(None, None)
  params = None if _ALGO.value == "ppo" else sac_checkpoint.load(_CHECKPOINT.value)
  out_dir = epath.Path(_OUT_DIR.value)
  out_dir.mkdir(parents=True, exist_ok=True)

  scene_option = mujoco.MjvOption()
  scene_option.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = False
  scene_option.flags[mujoco.mjtVisFlag.mjVIS_PERTFORCE] = False
  scene_option.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = False

  sac_params = manipulation_params.brax_sac_config("LeapCubeReorient")
  net_kwargs = dict(sac_params.network_factory)
  net_kwargs["hidden_layer_sizes"] = tuple(int(x) for x in _HIDDEN.value)

  conds = _FACES.value if _is_faceup() else _ANGLES.value
  for raw in conds:
    cond = int(raw) if _is_faceup() else float(raw)
    env = _make_env(cond)
    network = functools.partial(sac_networks.make_sac_networks, **net_kwargs)(
        observation_size=env.observation_size["state"][0],
        action_size=env.action_size,
        preprocess_observations_fn=running_statistics.normalize,
    )
    use_params = params
    if _DISTILLED.value:
      # 증류 정책은 flax 직렬화(장치 독립). 같은 구조 템플릿으로 되돌린다.
      from etils import epath as _epath
      import flax as _flax
      tmpl = {"norm": params[0],
              "policy": network.policy_network.init(jax.random.PRNGKey(0))}
      restored = _flax.serialization.from_bytes(
          tmpl, _epath.Path(_DISTILLED.value).resolve().read_bytes())
      use_params = (restored["norm"], restored["policy"])
    if _ALGO.value == "ppo":
      # PPO 정책은 dict 관측을 그대로 받는다(평탄화 전).
      inference = ppo_checkpoint.load_policy(_CHECKPOINT.value, deterministic=True)
    else:
      inference = jax.jit(
          sac_networks.make_inference_fn(network)(use_params, deterministic=True)
      )
    reset = jax.jit(env.reset)
    step = jax.jit(env.step)

    traj, notes = [], []
    for ep in range(_EPISODES.value):
      key = jax.random.PRNGKey(_SEED.value + ep * 7919)
      state = reset(key)
      ep_traj = [state]
      max_hold = 0.0
      goals = 0.0
      for t in range(_MAX_STEPS.value):
        key, act_key = jax.random.split(key)
        obs = state.obs if _ALGO.value == "ppo" else state.obs["state"]
        action, _ = inference(obs, act_key)
        state = step(state, action)
        ep_traj.append(state)
        max_hold = max(max_hold, float(state.metrics["success_hold_steps"]))
        if "goal_count" in state.metrics:
          goals = max(goals, float(state.metrics["goal_count"]))
        if float(state.done) > 0.5:
          break
      # 연쇄 모드에서는 성공이 종료가 아니므로 max_hold 가 hold 값에서 초기화된다.
      # 대표 수치는 "몇 번 이어갔는가"다.
      ok = goals >= 1.0 if _CHAIN.value else max_hold >= 10.0
      notes.append((ep, len(ep_traj) - 1, ok, goals))
      # 결과가 보이도록 마지막 자세를 잠깐 정지시킨다.
      traj.extend(ep_traj + [ep_traj[-1]] * _TAIL_FRAMES.value)

    frames = env.render(
        traj, height=_HEIGHT.value, width=_WIDTH.value, scene_option=scene_option
    )
    # env.dt 간격으로 시뮬레이션된 프레임을 목표 FPS 로 리샘플한다.
    duration = len(traj) * env.dt
    target = max(1, int(round(duration * _FPS.value)))
    idx = np.linspace(0, len(frames) - 1, target)
    frames = [frames[int(round(j))] for j in idx]

    if _is_faceup():
      label = _FACE_NAMES.get(cond, str(cond))
      path = out_dir / f"face_{label}.mp4"
    else:
      label = f"{cond*180:.0f}deg"
      path = out_dir / f"goal_{int(cond*180)}deg.mp4"
    media.write_video(path, frames, fps=_FPS.value)
    ok_n = sum(1 for _, _, ok, _ in notes if ok)
    extra = ("  연쇄 " + ", ".join(f"{g:.0f}회" for _, _, _, g in notes)
             if _CHAIN.value else "")
    print(f"{label:>8}  -> {path.name}  "
          f"({ok_n}/{len(notes)} 성공, 스텝 "
          + ", ".join(f"{n}{'O' if ok else 'X'}" for _, n, ok, _ in notes)
          + ")" + extra)


if __name__ == "__main__":
  app.run(main)
