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
