"""과제 2: 큐브를 굴려 원하는 면이 위를 보게 하고 손바닥 정중앙으로 되돌린다.

과제 1(CubeReorient)과 두 가지가 다르다.

1. **목표가 면 단위다.** 전체 자세(사원수)를 맞출 필요 없이 지정된 면의 법선이
   위를 향하기만 하면 된다. 그 법선 둘레의 회전(스핀)은 자유다. 즉 목표가
   SO(3) 의 한 점이 아니라 1차원 궤도이고, 과제 1보다 목표 집합이 넓다.

2. **위치 제약이 성공 조건에 들어간다.** 큐브가 손바닥 정중앙에서
   position_threshold 안에 있어야 한다. 과제 1 에서 위치는 shaping 보상일 뿐
   성공 판정에 들어가지 않았다.

구현 메모
  과제 1 의 `_cube_orientation_error` 는 성공 판정·progress 보상·지표·관측을
  모두 관통하는 단일 지점이다. 여기서는 그 함수만 "면 오차와 중심 오차의
  결합"으로 바꾸면 나머지 기계장치가 그대로 동작한다.

    task_error = max(face_error, center_error * thr / position_threshold)

  max 형태라 `task_error < success_threshold` 가 곧
  `face_error < thr AND center_error < position_threshold` 와 같다.
  두 조건의 AND 를 단일 스칼라로 표현하면서, 더 나쁜 쪽에 gradient 가 걸린다.
  위치 쪽 조밀 신호는 기존 `position` 보상 항이 따로 준다.

  목표 면은 별도 상태로 들고 다니지 않고 **mocap_quat 에 인코딩**한다.
  목표 자세 R_goal 에 대해 위를 보게 되는 면의 국소 법선은 n = R_goal^-1 · up
  이므로, data 만으로 목표 면을 복원할 수 있다. reset 이 info 를 만들기 전에도
  오차를 계산할 수 있어야 하는 제약과 맞물려 이 방식이 편하다.
"""

from typing import Any, Dict, Optional, Union

import jax
import jax.numpy as jp
import numpy as np
from ml_collections import config_dict
from mujoco import mjx
from mujoco.mjx._src import math

from mujoco_playground._src import mjx_env
from mujoco_playground._src.manipulation.leap_hand import reorient


_UP = jp.array([0.0, 0.0, 1.0])
_S2 = 0.70710678

# 각 면이 위를 보게 하는 목표 자세 6가지 (w, x, y, z).
# 순서: +z, -z, +x, -x, +y, -y 가 위로 오는 자세.
# 큐브 시작 자세가 항등(=+z 가 위)이므로 인덱스 0 은 회전이 필요 없는 자명한 경우다.
_FACE_GOAL_QUATS = jp.array([
    [1.0, 0.0, 0.0, 0.0],       # +z 면이 위 (시작 자세와 동일)
    [0.0, 1.0, 0.0, 0.0],       # -z 면이 위 (x축 180도)
    [_S2, 0.0, -_S2, 0.0],      # +x 면이 위 (y축 -90도)
    [_S2, 0.0, _S2, 0.0],       # -x 면이 위 (y축 +90도)
    [_S2, _S2, 0.0, 0.0],       # +y 면이 위 (x축 +90도)
    [_S2, -_S2, 0.0, 0.0],      # -y 면이 위 (x축 -90도)
], dtype=float)


def default_config() -> config_dict.ConfigDict:
  """과제 1 설정을 물려받고 이 과제의 항목을 얹는다."""
  config = reorient.default_config()
  # 면 법선이 위에서 벗어난 각도의 허용치(rad). 과제 요건은 "원하는 면이 정확하게
  # 위로" 이므로 0.15 rad(8.6도) 로 둔다. 참고로 0.785(45도)는 두 면이 똑같이
  # 위를 향하는 경계라 "어느 면이 위냐"를 겨우 가르는 수준이고, 0.6(34도)은
  # 눈으로도 확연히 기울어 보인다.
  #
  # 학습 주의: 이 값은 shaping 구간도 좁힌다(_reward_cube_orientation 의
  # margin = clip(thr*4, ...)). 과제 1 에서 임계값을 0.75 -> 0.40 으로 한 번에
  # 조였다가 성공률이 35.2% -> 19.5% 로 무너졌고, 0.8 -> 0.7 처럼 단계적으로
  # 조였을 때는 손실 없이 정밀도만 올랐다. 따라서 학습은 느슨한 값에서 시작해
  # 단계적으로 이 값까지 좁힌다(run_faceup_ppo.sh 의 STAGES).
  config.success_threshold = 0.15
  # 손바닥 정중앙에서 허용하는 거리(m). 실측 근거:
  #   에피소드 시작 시 중심 오차가 평균 17.8mm, 최대 30.6mm 라 30mm 로 두면
  #   대부분의 에피소드가 시작부터 위치 조건을 충족해 제약이 무의미해진다.
  #   무작위 행동으로 굴려도 에피소드당 최소 중심 오차의 중앙값이 9.2mm 이므로
  #   15mm 는 도달 가능하면서 시작 상태로는 대개 충족되지 않는 수준이다.
  config.position_threshold = 0.015
  # 위치 조건의 기준점.
  #   "palm"  : 손바닥 정중앙(고정점)까지의 거리. 기존 동작.
  #   "start" : **그 에피소드의 큐브 시작 위치**까지의 거리.
  #
  # "start" 는 과제를 "손바닥으로 당겨오기"에서 "제자리로 돌려놓기"로 바꾼다.
  # 요구는 "사람이 큐브 위치를 고쳐주지 않아도 바로 다음 목표면으로 넘어갈 수
  # 있는가"이고, 그건 정의상 "받은 자리에 돌아왔는가"다.
  #
  # 실측 근거 (2026-08-29, v48_A 정책, 5면 x 128환경)
  #   안착 직후 큐브는 손바닥 중앙에서 평균 28.4mm 떨어져 있다. 즉 기존
  #   "손바닥 15mm" 조건은 시작 지점보다 13mm 더 당겨오라는 요구였고, 시작
  #   상태로 충족되는 경우는 2.5% 뿐이었다.
  #   반대로 면을 맞춘 시점의 시작점 대비 이동량은 평균 22.9mm 이고 그중
  #   10mm 이내는 4.4% 뿐이다. 즉 "start" + 10mm 는 우연히 충족되지 않으면서
  #   이동량 분포에서 크게 벗어나지도 않는 수준이다.
  config.position_reference = "palm"
  # >=0 이면 목표 면을 고정한다(진단용). -1 이면 무작위.
  config.target_face = -1
  # 면별 샘플링 가중치 6개(+z,-z,+x,-x,+y,-y). 기본은 전부 1.0 = 균등.
  # 균등 샘플링은 난이도가 4배 차이나는 면에 같은 예산을 준다. 실측(2026-08-21)
  # 으로 +x 성공률이 51.2%, 나머지 네 면이 90~97% 이므로 +x 에 예산을 더 준다.
  # include_start_face=False 면 +z 가중치는 무시된다.
  config.face_weights = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
  # True 면 목표 면 원-핫(6)을 관측 뒤에 붙인다. 기본 관측은 목표를 "현재 자세와
  # 목표의 차이"(cube_ori_error_history)로만 담아, 정책이 **어느 면 과제인지**를
  # 직접 읽을 수 없다. 실측상 하나의 정책이 5면을 동시에 유지하지 못하고
  # 학습 중 잘하는 면이 뒤바뀌므로(v29: 같은 실행에서 +x·+y 가 +78%p 오르는 동안
  # -x 가 -48%p), 과제 식별자를 명시적으로 주는 것이 처방 후보다.
  # 관측 차원이 108->114, 179->185 로 바뀌어 이전 체크포인트와 호환되지 않는다.
  config.face_onehot_obs = False
  # False 면 시작 자세와 같은 +z 면(회전 불필요)을 목표에서 제외한다.
  config.include_start_face = False
  # 위치가 성공 조건에 들어가므로 중심 복귀 신호를 과제 1보다 키운다.
  config.reward_config.scales.position = 4.0
  return config


class CubeFaceUp(reorient.CubeReorient):
  """지정된 면을 위로 향하게 하고 손바닥 중앙으로 되돌린다."""

  def __init__(
      self,
      config: config_dict.ConfigDict = default_config(),
      config_overrides: Optional[Dict[str, Union[str, int, list[Any]]]] = None,
  ):
    super().__init__(config=config, config_overrides=config_overrides)

  # --- 과제 정의 ---------------------------------------------------------

  def _target_face_normal(self, data: mjx.Data) -> jax.Array:
    """목표 자세에서 위를 보게 되는 면의 국소 법선 (= R_goal^-1 · up).

    목표는 센서가 아니라 mocap 에서 읽는다(D2). 센서를 쓰면 목표 교체 직후
    한 스텝 동안 옛 목표의 법선이 나와 face_error 가 틀린다.
    """
    goal_quat = self._goal_quat(data)
    return math.rotate(_UP, math.quat_inv(goal_quat))

  def _face_error(self, data: mjx.Data) -> jax.Array:
    """목표 면 법선이 위(+z)에서 벗어난 각도(rad). 스핀에는 불변이다."""
    n_local = self._target_face_normal(data)
    n_world = math.rotate(n_local, self.get_cube_orientation(data))
    return jp.arccos(jp.clip(jp.dot(n_world, _UP), -1.0, 1.0))

  def _center_error(self, data: mjx.Data, info=None) -> jax.Array:
    """위치 조건의 오차(m). 기준점은 config.position_reference 가 정한다.

    "start" 인데 info 가 없으면(예: 리셋 전 진단 호출) 오차 0 으로 본다.
    시작 시점에는 정의상 큐브가 시작 위치에 있기 때문이다.
    """
    if self._config.position_reference == "start":
      # 계산 불가면 예외를 던진다. 기본값(0)을 돌려주면 "위치 조건이 항상 만족"
      # 으로 보여 조용히 틀린 결과가 나온다(N4).
      if info is None or "start_cube_pos" not in info:
        raise ValueError(
            'position_reference="start" 인데 info["start_cube_pos"] 가 없다. '
            "reset() 을 거치지 않은 data 로 호출했을 가능성이 크다.")
      return jp.linalg.norm(info["start_cube_pos"] - self.get_cube_position(data))
    return jp.linalg.norm(
        self.get_palm_position(data) - self.get_cube_position(data)
    )

  def _cube_orientation_error(self, data: mjx.Data, info=None) -> jax.Array:
    """면 오차와 위치 오차를 하나의 스칼라로 결합한다 (상세는 모듈 docstring).

    max 이므로 둘 중 하나라도 임계를 넘으면 즉시 실패로 떨어진다. 즉
    "면을 맞춘 뒤 옮긴다"는 순차 달성이 아니라 **동시 만족**을 요구한다.
    """
    thr = self._config.success_threshold
    pos_thr = self._config.position_threshold
    scaled_center = self._center_error(data, info) * (thr / pos_thr)
    return jp.maximum(self._face_error(data), scaled_center)

  # --- 목표 면 샘플링 ----------------------------------------------------

  def _sample_face_goal(self, rng: jax.Array) -> jax.Array:
    if self._config.target_face >= 0:
      idx = jp.asarray(self._config.target_face, dtype=jp.int32)
    else:
      lo = 0 if self._config.include_start_face else 1
      # 판정과 상수 준비는 전부 호스트에서 한다. jp 로 만들면 eval_shape 로
      # 관측 크기를 재는 경로에서 추상값이 되어 float() 이 터진다.
      w = [float(x) for x in self._config.face_weights]
      if all(x == w[lo] for x in w[lo:]):
        # 균등이면 기존 경로를 그대로 쓴다. 난수 소비가 달라지면 예전 실행이
        # 재현되지 않으므로 여기를 categorical 로 통일하지 않는다.
        idx = jax.random.randint(rng, (), lo, _FACE_GOAL_QUATS.shape[0])
      else:
        # 제외된 면과 가중치 0 인 면은 -inf 로 눌러 한 번의 categorical 로 뽑는다.
        logits = np.full(len(w), -np.inf, dtype=np.float32)
        for i in range(lo, len(w)):
          if w[i] > 0.0:
            logits[i] = np.log(w[i])
        idx = jax.random.categorical(rng, jp.asarray(logits))
    return _FACE_GOAL_QUATS[idx]

  def _up_face_index(self, data: mjx.Data) -> jax.Array:
    """지금 **실제로 위를 향한** 면의 인덱스. 목표 면(_face_index)과 다르다.

    연쇄 모드에서 같은 면이 다시 뽑히면 큐브가 이미 정렬돼 있어 즉시 성공하고,
    그것이 반복되면 연속 횟수가 무의미해진다. 그래서 이 면을 제외하고 뽑는다.
    큐브 자세에 의존하는 추적값이므로 호스트가 아니라 jax 로 계산해야 한다.
    """
    # 면 규약과 같은 순서의 국소 법선: +z, -z, +x, -x, +y, -y
    normals = jp.array([
        [0.0, 0.0, 1.0], [0.0, 0.0, -1.0],
        [1.0, 0.0, 0.0], [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0], [0.0, -1.0, 0.0],
    ], dtype=float)
    q = self.get_cube_orientation(data)
    world = jax.vmap(lambda n: math.rotate(n, q))(normals)
    return jp.argmax(world @ _UP)

  def _chain_next_goal(self, data: mjx.Data, info, achieved) -> mjx.Data:
    """성공 시 새 목표면을 뽑아 이어간다.

    갱신해야 하는 것들 (하나라도 빠지면 보상 해킹 구멍이 된다)
      prev_ori_error / best_ori_error : 새 목표 기준으로 다시 잰다. 그대로 두면
        progress(scale 35)와 best_progress(scale 80)가 옛 목표 기준의 낮은 값과
        새 목표의 큰 오차를 비교해 **한 스텝에 거대한 보상**을 낸다.
      success_hold_steps / best_hold_steps : 0. 남겨두면 새 목표를 즉시 달성한
        것으로 처리되고, hold_stairs 의 계단도 이미 통과된 것으로 막힌다.
      steps_since_goal : 0. 정체 판정의 기준이다.
      cube_ori_error_history : 새 목표 기준 값으로 **채운다**(roll 이 아니라 fill).
        3프레임 이력에 옛 목표 값이 남으면 3스텝 동안 잘못된 목표를 본다.
    """
    info["steps_since_goal"] = jp.where(achieved, 0, info["steps_since_goal"] + 1)
    info["goal_count"] = info["goal_count"] + achieved.astype(int)

    info["rng"], gk = jax.random.split(info["rng"])
    # 지금 위를 향한 면을 -inf 로 눌러 제외한다. include_start_face=False 면
    # +z(인덱스 0)도 함께 제외한다.
    lo = 0 if self._config.include_start_face else 1
    w = [float(x) for x in self._config.face_weights]
    base = np.full(len(w), -np.inf, dtype=np.float32)
    for i in range(lo, len(w)):
      if w[i] > 0.0:
        base[i] = np.log(w[i])
    logits = jp.asarray(base).at[self._up_face_index(data)].set(-jp.inf)
    new_quat = _FACE_GOAL_QUATS[jax.random.categorical(gk, logits)]

    quat = jp.where(achieved,
                    jp.broadcast_to(new_quat, data.mocap_quat.shape),
                    data.mocap_quat)
    data = data.replace(mocap_quat=quat)

    err = self._cube_orientation_error(data, info)
    info["prev_ori_error"] = jp.where(achieved, err, info["prev_ori_error"])
    info["best_ori_error"] = jp.where(achieved, err, info["best_ori_error"])
    info["success_hold_steps"] = jp.where(achieved, 0, info["success_hold_steps"])
    info["best_hold_steps"] = jp.where(achieved, 0, info["best_hold_steps"])
    # 이력을 새 목표 기준으로 채운다(옛 목표 잔상 제거).
    # 주의: _get_obs 는 노이즈 섞인 큐브 자세로 이력을 굴리는데 여기 채움값은
    # 깨끗한 자세를 쓴다. 교체 직후 _get_obs 가 곧바로 새 프레임을 굴려 넣으므로
    # 차이가 남는 것은 뒤쪽 2프레임뿐이고, 관측 노이즈 크기에 비해 작다.
    xmat = self._goal_relative_xmat(data)
    info["cube_ori_error_history"] = jp.where(
        achieved,
        jp.tile(xmat, self._config.history_len),
        info["cube_ori_error_history"])
    return data

  def _face_index(self, data: mjx.Data) -> jax.Array:
    """mocap_quat 을 6개 목표 자세 중 가장 가까운 인덱스로 되돌린다.

    `get_cube_goal_orientation` 은 **센서**를 읽으므로 쓰면 안 된다. reset 은
    mocap_quat 을 교체한 뒤 물리 forward 를 돌지 않아 센서가 아직 이전 값이고,
    그러면 목표 면과 무관하게 늘 같은 인덱스가 나온다(실측: 전 면에서 2).
    mocap_quat 을 직접 읽으면 reset 시점에도 정확하다.
    """
    q = self._goal_quat(data)
    return jp.argmin(jp.sum((_FACE_GOAL_QUATS - q[None, :]) ** 2, axis=-1))

  def _get_obs(self, data: mjx.Data, info: dict[str, Any]) -> mjx_env.Observation:
    obs = super()._get_obs(data, info)
    if not self._config.face_onehot_obs:
      return obs
    onehot = jax.nn.one_hot(self._face_index(data), _FACE_GOAL_QUATS.shape[0])
    if isinstance(obs, dict):
      return {k: jp.concatenate([v, onehot]) for k, v in obs.items()}
    return jp.concatenate([obs, onehot])

  # --- reset / step ------------------------------------------------------

  def reset(self, rng: jax.Array) -> mjx_env.State:
    rng, face_rng = jax.random.split(rng)
    state = super().reset(rng)

    # 과제 1 이 뽑은 목표 자세를 "면이 위를 보는" 자세로 교체한다.
    goal_quat = self._sample_face_goal(face_rng)
    data = state.data.replace(
        mocap_quat=jp.broadcast_to(goal_quat, state.data.mocap_quat.shape)
    )

    # 목표가 바뀌었으므로 오차 기준값과 관측을 다시 계산한다.
    info = dict(state.info)
    err = self._cube_orientation_error(data, info)
    info["prev_ori_error"] = err
    info["best_ori_error"] = err
    obs = self._get_obs(data, info)

    metrics = dict(state.metrics)
    metrics["face_error"] = jp.zeros(())
    metrics["center_error"] = jp.zeros(())
    metrics["face_ok"] = jp.zeros(())
    metrics["center_ok"] = jp.zeros(())
    return state.replace(data=data, obs=obs, info=info, metrics=metrics)

  def step(self, state: mjx_env.State, action: jax.Array) -> mjx_env.State:
    state = super().step(state, action)
    # 두 조건 중 어느 쪽이 병목인지 보려면 결합 전 값을 따로 봐야 한다.
    face = self._face_error(state.data)
    center = self._center_error(state.data, state.info)
    state.metrics["face_error"] = face
    state.metrics["center_error"] = center
    state.metrics["face_ok"] = (
        face < self._config.success_threshold
    ).astype(float)
    state.metrics["center_ok"] = (
        center < self._config.position_threshold
    ).astype(float)
    return state


def domain_randomize(model: mjx.Model, rng: jax.Array):
  return reorient.domain_randomize(model, rng)
