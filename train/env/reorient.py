# Copyright 2025 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================
"""Reorient task for leap hand."""

from typing import Any, Dict, Optional, Union

import jax
import jax.numpy as jp
from ml_collections import config_dict
from mujoco import mjx
from mujoco.mjx._src import math
import numpy as np

from mujoco_playground._src import mjx_env
from mujoco_playground._src import reward
from mujoco_playground._src.manipulation.leap_hand import base as leap_hand_base
from mujoco_playground._src.manipulation.leap_hand import leap_hand_constants as consts


def default_config() -> config_dict.ConfigDict:
  return config_dict.create(
      ctrl_dt=0.05,
      sim_dt=0.01,
      action_scale=0.2,
      action_repeat=1,
      ema_alpha=1.0,
      episode_length=1000,
      goal_angle_scale=1.0,
      # >0 이면 에피소드마다 [goal_angle_scale_min, goal_angle_scale] 에서 목표각을
      # 균일 샘플링한다. 0 이면 goal_angle_scale 로 고정(기존 동작).
      goal_angle_scale_min=0.0,
      # 목표각을 [min, scale] 구간에서 Beta(a, b) 로 뽑는다. 성공률이 낮은 구간을
      # 더 자주 보게 하되 어느 구간도 버리지 않으므로(전 구간 지지), 구간을 통째로
      # 갈아치우는 커리큘럼과 달리 망각이 생기지 않는다.
      #   a=b=1 (기본값) -> 균일 샘플링. 기존 동작과 완전히 동일하다.
      #   goal_angle_bias>0 -> a 에 더해져 Beta(1+bias, 1), 즉 u**(1/(1+bias)) 로
      #     큰 각도 쪽으로 기울인다.
      #   a,b>1 -> 가운데가 볼록해져 중간 각도를 더 자주 뽑는다.
      goal_angle_bias=0.0,
      goal_angle_beta_a=1.0,
      goal_angle_beta_b=1.0,
      # >=0 이면 목표 회전축을 그 인덱스로 고정한다(0~2 면축, 3~8 모서리축).
      # 학습에는 쓰지 않고 각도별 난이도의 원인을 가르는 진단용이다. -1 이면 무작위.
      goal_axis_index=-1,
      # 모터 보호: 구동 범위의 양 끝을 이 비율만큼 잘라내고 쓴다.
      # 0.2 면 하한/상한에서 각각 10%씩 물러난 범위만 명령한다. 실기에서
      # 기계적 한계에 붙어 구동하면 감속기·모터에 무리가 가므로 여유를 둔다.
      # 0(기본값)이면 기존 동작(하드웨어 범위 전체 사용)과 동일하다.
      joint_limit_margin=0.0,
      # 손가락 pip / dip 의 **하한만** 이 값으로 올린다(과신전 금지). 상한은 그대로.
      # 왜 (2026-08-30 실측)
      #   if_dip 이 시간의 46.4% 를 하한 -0.366rad(-21도)에 붙어 있다. 사람 손가락은
      #   끝마디 능동 과신전이 사실상 0도이고 수동으로도 5~10도다. 즉 정책이
      #   **해부학적으로 사람이 못 만드는 자세**를 절반 가까이 쓰고 있다.
      #   보상으로는 안 고쳐졌다(hand_pose 를 500배 올려도 무변화, action_rate -3.0
      #   은 낙하율 66% 로 정책을 파괴). 그 자세가 물리적으로 유리하기 때문이다.
      #   -> 보상이 아니라 **범위로** 막는다.
      # 근거
      #   v49 에서 joint_limit_margin=0.1(양끝 5%씩 제거)은 무비용이었다
      #   (98.36 -> 98.98%). 여기서는 한쪽 끝만 자르므로 손실이 더 작을 것으로 본다.
      # -9.0 이면 끔(기존 동작).
      hyperext_floor=-9.0,
      # 작동 지연: 정책이 낸 명령이 K 스텝 뒤에 모터에 도달한다고 본다.
      # 0 이면 끔(상류와 동일).
      #
      # 왜 (sim2real)
      #   시뮬은 관측->행동->물리가 같은 틱에 일어난다. 실기 LEAP Hand 는
      #   Dynamixel 을 USB 시리얼로 물리므로 관측 읽기, 정책 추론, 명령 쓰기가
      #   각각 지연을 갖는다. 이 지연을 학습에서 겪지 않은 정책은 실기에서
      #   한 틱 늦은 피드백을 받고 진동한다(DeXtreme 2210.13702 이 같은 이유로
      #   action latency 를 무작위화한다).
      #
      # 어떻게
      #   에피소드마다 d ~ U{0..K} 를 뽑아 고정한다. **에피소드 내에서 고정**인
      #   이유는, 매 스텝 다시 뽑으면 명령 순서가 뒤바뀌어(추월) 실기에 없는
      #   현상을 학습시키기 때문이다. 실기의 지연은 느리게 변하는 값이다.
      #
      # 주의
      #   지연은 **물리에만** 건다. 관측의 last_act 와 action_rate 보상은 정책
      #   자신의 출력을 그대로 본다. 정책이 자기가 내지도 않은 과거 행동으로
      #   벌점을 받으면 평활 항의 의미가 무너진다.
      action_delay_steps=0,
      # 리셋 직후 정책 없이 물리만 굴려 큐브를 손 위에 **안착**시킬 스텝 수.
      # 0 이면 상류와 같은 동작(큐브가 공중 42mm 에서 시작해 떨어진다).
      #
      # 왜 필요한가 (2026-08-29 실측)
      #   리셋 시점의 접촉이 0개다. 큐브는 42mm 낙하하고 x 로 26mm 미끄러진 뒤
      #   0.10~0.15초(2~3스텝)만에 안착한다. 실기에서는 사람이 큐브를 손에
      #   올려주므로 이 낙하는 시뮬에만 존재하는 초기 충격이다.
      #   성능 기여도 없다: 교사 평가에서 낙하/안착3/안착10 이 99.30% 로 동일했다.
      #   위치를 상수로 추정하는 대신 물리로 안착시키는 이유는, 손 자세가
      #   무작위화(+-0.1 rad)되어 안착점이 매번 달라지기 때문이다.
      settle_steps=0,
      # True 면 성공 시 에피소드를 끝내지 않고 **새 목표를 뽑아 이어간다**.
      # 근거: DeXtreme(2210.13702)·PTLD(2603.04531) 모두 목표 도달 시 리셋 없이
      # 목표를 재샘플링하고 "연속 달성 횟수"를 성공 지표로 쓴다. 이러면
      # "큐브를 재시작 가능한 자리에, 손을 회복 가능한 자세로 남겨라"를 사람이
      # 임계값으로 명시할 필요가 없다 - 다음 목표가 그것을 강제한다.
      chain_goals=False,
      # 한 목표에 이 스텝을 넘게 쓰면 정체로 보고 에피소드를 끝낸다.
      # 600 = 30초. 실측상 가장 느린 면(+x)이 최대 500스텝 걸리므로 그보다 위여야
      # 면 난이도가 아니라 이 설정 때문에 실패하는 일이 없다.
      goal_timeout_steps=600,
      # joint_limit 보상항의 기준. 0.75 면 가동 범위의 바깥 25% 부터 비용이 붙는다.
      joint_limit_soft_factor=0.75,
      # 모터 토크 제한(Nm). LEAP Hand 는 관절 각도가 아니라 **전류 제한**으로
      # 모터를 보호한다(공식 API 기본 300mA, Full 최대 550mA). 토크는 전류에
      # 비례하므로 XL330-M288-T 정지 토크 0.52Nm / 정지 전류 1.47A 기준으로
      #   300mA -> 0.11Nm,  550mA -> 0.19Nm
      # 이 모델은 기본적으로 forcerange 가 없어(토크 무제한) 정책이 실기에서
      # 낼 수 없는 토크를 전제하고 학습된다(실측: 명령의 24~31%가 정지 토크 초과).
      # 0(기본값)이면 제한 없음 = 기존 동작.
      actuator_force_limit=0.0,
      # True 면 success_reward 를 "성공 유지 달성" 시점에 한 번만 준다(착취 방지).
      # False(기본) 면 기존처럼 임계값 안에 있는 매 스텝마다 준다.
      hold_gated_success=False,
      # 게이트의 비용은 유지 길이에 비례해 커진다: 일시금이 도달 시점에서
      # gamma^H 로 깎이고(0.99^100=0.37), SAC 는 1-step TD 라 그 일시금을 H 번의
      # 벨만 갱신으로 거꾸로 밀어야 한다. PPO 는 unroll 40 + GAE 라 한 번에
      # 40스텝을 관통하므로 같은 gamma 에서도 문제가 없다.
      # hold_stairs>1 이면 일시금을 유지 구간에 균등 분할해 계단식으로 준다.
      # 총 지급액은 그대로라 착취 유인은 생기지 않고(각 계단이 실제 연속 유지를
      # 요구한다), 최대 전파 거리만 H -> H/hold_stairs 로 줄어든다.
      hold_stairs=1,
      # BraxAutoResetWrapper 는 done 시 data/obs 만 되돌리고 info 는 넘긴다.
      # 그래서 best_ori_error(에피소드 최소 오차)가 롤아웃 내내 누적돼, 가장
      # 무거운 보상항인 best_progress(스케일 80)가 env 당 첫 에피소드 이후
      # 죽는다. 실측(PPO 교사, 32env): 스텝당 0.3076 -> 0.00078 (394배 감소).
      # True 면 에피소드마다 초기화해 이 항을 되살린다. 보상 함수가 바뀌므로
      # 기존 실험과 비교가 끊긴다. 기본값 False 는 기존 동작 유지.
      reset_best_on_episode=False,
      success_threshold=0.1,
      success_hold_steps=20,
      history_len=3,
      obs_noise=config_dict.create(
          level=1.0,
          scales=config_dict.create(
              joint_pos=0.05,
              cube_pos=0.02,
              cube_ori=0.1,
          ),
          random_ori_injection_prob=0.0,
      ),
      reward_config=config_dict.create(
          scales=config_dict.create(
              orientation=20.0,
              progress=10.0,
              best_progress=50.0,
              direct_penalty=2.0,
              precision=0.0,
              fine_025=0.0,
              fine_020=0.0,
              position=0.5,
              termination=-20.0,
              hand_pose=-0.001,
              action_rate=-0.005,
              joint_vel=-0.001,
              energy=-2e-4,
              # 관절이 가동 범위 가장자리에 머무는 것을 벌한다. 시뮬에서는 무해하나
              # 실기에서는 기구가 스토퍼에 밀린다. 실측(2026-08-28, diag_joint_stress):
              # 교사가 SPiDR(2509.18648) 기준 "바깥 25%"에 48.9% 의 시간을 머물고,
              # 낸 목표의 15.8% 가 URDF 한계로 잘린다. 증류 정책이 그대로 물려받는다.
              # 0.0 이면 끔(기존 동작과 동일).
              joint_limit=0.0,
          ),
          success_reward=600.0,
          # 유지 달성 시 추가 일시금(0이면 미사용). 스텝당 성공 보상과 병행 가능.
          hold_bonus=0.0,
      ),
      pert_config=config_dict.create(
          enable=False,
          linear_velocity_pert=[0.0, 3.0],
          angular_velocity_pert=[0.0, 0.5],
          pert_duration_steps=[1, 100],
          pert_wait_steps=[60, 150],
      ),
      impl='warp',
      naconmax=30 * 8192,
      njmax=160,
  )


class CubeReorient(leap_hand_base.LeapHandEnv):
  """Reorient a cube to match a goal orientation."""

  def __init__(
      self,
      config: config_dict.ConfigDict = default_config(),
      config_overrides: Optional[Dict[str, Union[str, int, list[Any]]]] = None,
  ):
    super().__init__(
        xml_path=consts.CUBE_XML.as_posix(),
        config=config,
        config_overrides=config_overrides,
    )
    self._post_init()

  def _post_init(self) -> None:
    home_key = self._mj_model.keyframe("home")
    self._init_q = jp.array(home_key.qpos, dtype=float)
    self._init_mpos = jp.array(home_key.mpos, dtype=float)
    self._init_mquat = jp.array(home_key.mquat, dtype=float)
    self._lowers = self._mj_model.actuator_ctrlrange[:, 0]
    self._uppers = self._mj_model.actuator_ctrlrange[:, 1]
    self._hand_qids = mjx_env.get_qpos_ids(self.mj_model, consts.JOINT_NAMES)
    self._hand_dqids = mjx_env.get_qvel_ids(self.mj_model, consts.JOINT_NAMES)
    self._cube_qids = mjx_env.get_qpos_ids(self.mj_model, ["cube_freejoint"])
    self._floor_geom_id = self._mj_model.geom("floor").id
    self._cube_geom_id = self._mj_model.geom("cube").id
    self._cube_body_id = self._mj_model.body("cube").id
    self._cube_mass = self._mj_model.body_subtreemass[self._cube_body_id]
    self._default_pose = self._init_q[self._hand_qids]
    # 구동 범위를 양 끝에서 margin/2 씩 잘라낸 '안전 범위'.
    margin = self._config.joint_limit_margin
    center = 0.5 * (self._lowers + self._uppers)
    half = 0.5 * (self._uppers - self._lowers) * (1.0 - margin)
    self._safe_lowers = center - half
    self._safe_uppers = center + half
    # 과신전 차단: 손가락 pip/dip 의 하한만 올린다.
    # 관절 순서는 consts.JOINT_NAMES = if_mcp,if_rot,if_pip,if_dip, mf_..., rf_...,
    # th_cmc,th_axl,th_mcp,th_ipl 이고 pip/dip 은 각 손가락의 3,4번째다.
    # 엄지(th_ipl)는 사람도 IP 과신전이 10~20도 가능하므로 제외한다.
    floor = float(self._config.hyperext_floor)
    if floor > -8.0:
      # _safe_lowers 는 numpy 배열이다(actuator_ctrlrange 에서 온다). jax 의
      # .at[] 이 없으므로 numpy 로 처리한다. 모델 상수라 추적 대상이 아니다.
      import numpy as _np
      sl = _np.array(self._safe_lowers, dtype=float)
      idx = [2, 3, 6, 7, 10, 11]
      sl[idx] = _np.maximum(sl[idx], floor)
      self._safe_lowers = sl
    # 토크 제한은 mjx 모델에 반영돼야 하므로 mj_model 을 고친 뒤 다시 올린다.
    force_limit = self._config.actuator_force_limit
    if force_limit > 0.0:
      self._mj_model.actuator_forcerange[:, 0] = -force_limit
      self._mj_model.actuator_forcerange[:, 1] = force_limit
      self._mj_model.actuator_forcelimited[:] = 1
      self._mjx_model = mjx.put_model(
          self._mj_model, impl=self._config.impl
      )

  def reset(self, rng: jax.Array) -> mjx_env.State:
    # Randomize the goal orientation.
    rng, goal_rng = jax.random.split(rng)
    # 목표: 면이 반듯하게 정렬된 180도 회전 9가지 중 무작위 선택
    s2 = 0.70710678
    aligned = jp.array([
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, s2, s2, 0.0],
        [0.0, s2, -s2, 0.0],
        [0.0, s2, 0.0, s2],
        [0.0, s2, 0.0, -s2],
        [0.0, 0.0, s2, s2],
        [0.0, 0.0, s2, -s2],
    ], dtype=float)
    # goal_axis_index >= 0 이면 목표 회전축을 고정한다(진단용). 기본 -1 은 무작위.
    # 9개 축은 면축 3개(x/y/z)와 모서리축 6개로 나뉘고, 큐브 대칭군과의 관계가
    # 달라서 각도별 난이도를 가르는지 확인하려면 축을 분리해 재야 한다.
    if self._config.goal_axis_index >= 0:
      idx = jp.asarray(self._config.goal_axis_index, dtype=jp.int32)
    else:
      idx = jax.random.randint(goal_rng, (), 0, 9)
    goal_axis = aligned[idx, 1:]
    # goal_angle_scale_min > 0 이면 에피소드마다 목표 회전각을
    # [goal_angle_scale_min, goal_angle_scale] 에서 균일 샘플링한다.
    # 스테이지로 각도를 고정하면 다음 스테이지 학습이 이전 정책을 망가뜨리는데
    # (v23 stage05~10 에서 반복 관측), 한 정책이 전 범위를 동시에 배우게 해서 피한다.
    # 0(기본값)이면 goal_angle_scale 로 고정 -- 기존 동작과 동일하다.
    if self._config.goal_angle_scale_min > 0.0:
      rng, angle_rng = jax.random.split(rng)
      # Beta(1,1) 은 균일분포지만 jax.random.beta 를 태우면 난수 소비가 달라져
      # 기존 실행과 재현이 어긋난다. 기본값일 때는 uniform 경로를 그대로 쓴다.
      a = self._config.goal_angle_beta_a + self._config.goal_angle_bias
      b = self._config.goal_angle_beta_b
      if a == 1.0 and b == 1.0:
        u = jax.random.uniform(angle_rng, ())
      else:
        u = jax.random.beta(angle_rng, a, b, ())
      lo = self._config.goal_angle_scale_min
      angle_scale = lo + (self._config.goal_angle_scale - lo) * u
    else:
      angle_scale = self._config.goal_angle_scale
    goal_angle = jp.pi * angle_scale
    goal_quat = jp.concatenate(
        [jp.array([jp.cos(goal_angle / 2.0)]), goal_axis * jp.sin(goal_angle / 2.0)]
    )

    # Randomize the hand pose.
    rng, pos_rng, vel_rng = jax.random.split(rng, 3)
    q_hand = jp.clip(
        self._default_pose + 0.1 * jax.random.normal(pos_rng, (consts.NQ,)),
        self._safe_lowers,
        self._safe_uppers,
    )
    v_hand = 0.0 * jax.random.normal(vel_rng, (consts.NV,))

    # Randomize the cube pose.
    rng, p_rng, quat_rng = jax.random.split(rng, 3)
    start_pos = jp.array([0.1, 0.0, 0.05]) + jax.random.uniform(
        p_rng, (3,), minval=-0.01, maxval=0.01
    )
    start_quat = jp.array([1.0, 0.0, 0.0, 0.0])  # 시작은 정자세 고정
    q_cube = jp.array([*start_pos, *start_quat])
    v_cube = jp.zeros(6)

    qpos = jp.concatenate([q_hand, q_cube])
    qvel = jp.concatenate([v_hand, v_cube])
    data = mjx_env.make_data(
        self._mj_model,
        qpos=qpos,
        ctrl=q_hand,
        qvel=qvel,
        mocap_pos=self._init_mpos,
        mocap_quat=goal_quat,
        impl=self._mjx_model.impl.value,
        naconmax=self._config.naconmax,
        njmax=self._config.njmax,
    )

    # 큐브를 손 위에 안착시킨다. 목표 자세를 유지하는 명령(ctrl=q_hand)만 준
    # 채 물리를 굴리므로 손은 제자리에 있고 큐브만 내려앉는다.
    for _ in range(int(self._config.settle_steps)):
      data = mjx_env.step(self.mjx_model, data, q_hand, self.n_substeps)

    rng, pert1, pert2, pert3 = jax.random.split(rng, 4)
    pert_wait_steps = jax.random.randint(
        pert1,
        (1,),
        minval=self._config.pert_config.pert_wait_steps[0],
        maxval=self._config.pert_config.pert_wait_steps[1],
    )
    pert_duration_steps = jax.random.randint(
        pert2,
        (1,),
        minval=self._config.pert_config.pert_duration_steps[0],
        maxval=self._config.pert_config.pert_duration_steps[1],
    )
    pert_lin = jax.random.uniform(
        pert3,
        minval=self._config.pert_config.linear_velocity_pert[0],
        maxval=self._config.pert_config.linear_velocity_pert[1],
    )
    pert_ang = jax.random.uniform(
        pert3,
        minval=self._config.pert_config.angular_velocity_pert[0],
        maxval=self._config.pert_config.angular_velocity_pert[1],
    )
    pert_velocity = jp.array([pert_lin] * 3 + [pert_ang] * 3)

    # 작동 지연 d ~ U{0..K}. 에피소드 내내 고정한다(명령 추월 방지).
    rng, delay_rng = jax.random.split(rng)
    action_delay = jax.random.randint(
        delay_rng, (), 0, self._config.action_delay_steps + 1
    )

    info = {
        "rng": rng,
        "action_delay": action_delay,
        "step": 0,
        "steps_since_last_success": 0,
        "success_hold_steps": 0,
        # 에피소드 내 최고 연속 유지 기록. 계단식 일시금(hold_stairs)을 계단당
        # 한 번만 지급하기 위한 것으로, 없으면 "짧게 유지하고 끊기"를 반복해
        # 같은 금액을 무한히 반복 수령할 수 있다.
        "best_hold_steps": 0,
        "success_count": 0,
        "last_act": jp.zeros(self.mjx_model.nu),
        "act_hist": jp.zeros(
            (self._config.action_delay_steps + 1, self.mjx_model.nu)
        ),
        "last_last_act": jp.zeros(self.mjx_model.nu),
        "motor_targets": data.ctrl,
        "qpos_error_history": jp.zeros(self._config.history_len * 16),
        "cube_pos_error_history": jp.zeros(self._config.history_len * 3),
        "cube_ori_error_history": jp.zeros(self._config.history_len * 6),
        "goal_quat_dquat": jp.zeros(3),
        # 에피소드 시작 시점의 큐브 위치. 과제 2 의 "제자리로 돌려놓기" 성공
        # 조건이 이것을 기준점으로 쓴다. 손바닥 중앙(고정점)과 달리 리셋마다
        # 다르므로(안착 위치 편차 +-12mm) info 에 담아 둔다.
        "start_cube_pos": self.get_cube_position(data),
        # 연쇄 모드: 지금까지 달성한 목표 수와 현재 목표에 쓴 스텝 수.
        "goal_count": jp.zeros((), dtype=int),
        "steps_since_goal": jp.zeros((), dtype=int),
        "prev_ori_error": self._cube_orientation_error(data),
        "best_ori_error": self._cube_orientation_error(data),
        # Perturbation.
        "pert_wait_steps": pert_wait_steps,
        "pert_duration_steps": pert_duration_steps,
        "pert_vel": pert_velocity,
        "pert_dir": jp.zeros(6, dtype=float),
        "last_pert_step": jp.array([-jp.inf], dtype=float),
    }

    metrics = {}
    for k in self._config.reward_config.scales.keys():
      metrics[f"reward/{k}"] = jp.zeros(())
    metrics["reward/success"] = jp.zeros((), dtype=float)
    metrics["steps_since_last_success"] = 0
    metrics["success_hold_steps"] = 0
    # 연쇄 지표. metrics 는 reset·step 양쪽에서 키 집합이 같아야 한다(N2) -
    # State 가 jit/scan 경계를 넘는 pytree 라 구조가 다르면 터진다.
    metrics["goal_count"] = 0.0
    metrics["steps_since_goal"] = 0.0
    # D7: 종료 사유. 평가에서 낙하/정체/시간초과를 갈라야 연쇄가 왜 끊겼는지 안다.
    metrics["term_dropped"] = 0.0
    metrics["term_stuck"] = 0.0
    metrics["success_count"] = 0
    metrics["orientation_error"] = 0.0
    metrics["best_orientation_error"] = 0.0
    metrics["orientation_error_lt_0_60"] = 0.0
    metrics["orientation_error_lt_0_40"] = 0.0
    metrics["orientation_error_lt_0_25"] = 0.0
    metrics["orientation_error_lt_0_20"] = 0.0
    metrics["orientation_error_lt_0_15"] = 0.0
    metrics["orientation_error_lt_0_12"] = 0.0
    metrics["orientation_error_lt_0_10"] = 0.0

    obs = self._get_obs(data, info)
    reward, done = jp.zeros(2)  # pylint: disable=redefined-outer-name
    return mjx_env.State(data, obs, reward, done, metrics, info)

  def step(self, state: mjx_env.State, action: jax.Array) -> mjx_env.State:
    if self._config.pert_config.enable:
      state = self._maybe_apply_perturbation(state, state.info["rng"])

    # 작동 지연. 버퍼 0번이 최신이고, 에피소드마다 고정된 d 를 꺼내 쓴다.
    # action(정책 출력)은 그대로 두고 exec_action 만 물리에 넘긴다.
    exec_action = action
    if self._config.action_delay_steps > 0:
      hist = jp.roll(state.info["act_hist"], 1, axis=0).at[0].set(action)
      state.info["act_hist"] = hist
      exec_action = hist[state.info["action_delay"]]

    # Apply control and step the physics.
    delta = exec_action * self._config.action_scale
    motor_targets = state.data.ctrl + delta
    motor_targets = jp.clip(motor_targets, self._safe_lowers, self._safe_uppers)
    motor_targets = (
        self._config.ema_alpha * motor_targets
        + (1 - self._config.ema_alpha) * state.info["motor_targets"]
    )

    data = mjx_env.step(
        self.mjx_model, state.data, motor_targets, self.n_substeps
    )
    state.info["motor_targets"] = motor_targets

    ori_error = self._cube_orientation_error(data, state.info)
    success = ori_error < self._config.success_threshold
    state.info["steps_since_last_success"] = jp.where(
        success, 0, state.info["steps_since_last_success"] + 1
    )
    state.info["success_hold_steps"] = jp.where(
        success, state.info["success_hold_steps"] + 1, 0
    )
    # BraxAutoResetWrapper 는 done 시 data/obs 만 되돌리고 info 는 그대로 넘긴다
    # (그 클래스 docstring 에 명시돼 있다). 따라서 에피소드 누적값은 환경이
    # 직접 지워야 한다. 지우지 않으면 env 당 첫 에피소드에서만 계단이 지급되고
    # 이후엔 best 가 이미 최댓값이라 영원히 0원이 된다(실측: 512env x 4계단
    # = 2,048건만 지급, 기대치의 1/4).
    # 새 에피소드의 첫 스텝은 info["steps"]==0 으로 식별한다. AutoReset 이 done
    # 에서 steps 를 0 으로 만든 뒤 EpisodeWrapper 가 내부 env.step 을 먼저 부르고
    # 그 다음에 증가시키므로, 여기서 보는 값이 0 이면 에피소드 첫 스텝이다.
    # 조기 종료와 800스텝 절단을 모두 덮는다.
    if "steps" in state.info:
      new_episode = state.info["steps"] == 0
      prev_best = jp.where(new_episode, 0, state.info["best_hold_steps"])
      # D3: BraxAutoResetWrapper 는 full_reset=False 에서 data/obs 만 되돌리고
      # info 는 그대로 넘긴다(wrapper.py 문서화된 동작). 연쇄 카운터를 여기서
      # 되돌리지 않으면 이월된다. steps_since_goal 은 목표 달성 시에만 0 이 되므로
      # 첫 에피소드가 목표를 못 채우고 절단되면 다음 에피소드가 이미 timeout 을
      # 넘긴 상태로 시작해 **첫 스텝에 종료**하고, 그 뒤로 회복되지 않는다.
      # 실측(episode_length=200, timeout=600): 32환경 중 18.8%가 매 스텝 종료,
      # 평균 에피소드 길이 10.1스텝. goal_count 도 에피소드를 넘어 누적돼
      # "연속 달성 횟수" 지표가 무의미해진다.
      # best_ori_error 에서 같은 사고가 있었고(394배 감소) 그 대응이
      # reset_best_on_episode 다. 여기는 조건 없이 항상 되돌린다 - 연쇄 카운터는
      # 정의상 에피소드 지역 값이다.
      state.info["steps_since_goal"] = jp.where(
          new_episode, 0, state.info["steps_since_goal"]
      )
      state.info["goal_count"] = jp.where(
          new_episode, 0, state.info["goal_count"]
      )
      if self._config.reset_best_on_episode:
        # 새 에피소드면 기록을 현재 오차로 되돌린다. reset() 이 넣는 초기값과
        # 같은 의미다(그 시점의 오차).
        state.info["best_ori_error"] = jp.where(
            new_episode, ori_error, state.info["best_ori_error"]
        )
    else:
      prev_best = state.info["best_hold_steps"]
    state.info["best_hold_steps"] = jp.maximum(
        prev_best, state.info["success_hold_steps"]
    )
    state.info["success_count"] = jp.where(
        success, state.info["success_count"] + 1, state.info["success_count"]
    )
    state.metrics["steps_since_last_success"] = state.info[
        "steps_since_last_success"
    ]
    state.metrics["success_hold_steps"] = state.info["success_hold_steps"]
    state.metrics["success_count"] = state.info["success_count"]
    state.metrics["orientation_error"] = ori_error
    state.metrics["best_orientation_error"] = jp.minimum(
        state.info["best_ori_error"], ori_error
    )
    state.metrics["orientation_error_lt_0_60"] = (ori_error < 0.60).astype(float)
    state.metrics["orientation_error_lt_0_40"] = (ori_error < 0.40).astype(float)
    state.metrics["orientation_error_lt_0_25"] = (ori_error < 0.25).astype(float)
    state.metrics["orientation_error_lt_0_20"] = (ori_error < 0.20).astype(float)
    state.metrics["orientation_error_lt_0_15"] = (ori_error < 0.15).astype(float)
    state.metrics["orientation_error_lt_0_12"] = (ori_error < 0.12).astype(float)
    state.metrics["orientation_error_lt_0_10"] = (ori_error < 0.10).astype(float)

    done = self._get_termination(data, state.info)

    # obs 는 목표 교체 **뒤에** 만든다. 순서가 반대면 관측이 옛 목표 기준으로
    # 계산되고, _get_obs 가 갱신하는 cube_ori_error_history(3프레임)에 옛 목표
    # 값이 남아 새 목표를 3스텝 동안 잘못 본다.
    rewards = self._get_reward(data, action, state.info, state.metrics, done)
    rewards = {
        k: v * self._config.reward_config.scales[k] for k, v in rewards.items()
    }
    reward = sum(rewards.values()) * self.dt  # pylint: disable=redefined-outer-name

    # 목표 고정: 에피소드 동안 목표를 갱신하지 않고 유지한다.
    # (연쇄 모드면 아래에서 조건부로 교체한다.)
    data = data.replace(mocap_quat=state.data.mocap_quat)
    state.metrics["reward/success"] = success.astype(float)
    # 스텝당 성공 보상은 착취 가능하다: 성공 유지 10스텝을 채우면 에피소드가
    # 종료돼 이후 보상이 끊기므로, 임계값 근처를 오가며 성공 스텝만 쌓고
    # 절대 유지하지 않는 것이 보상 최대화 전략이 된다. PPO 300M(v28)이 실제로
    # 이 행동을 학습했다(성공 스텝 359개, 최대 연속 유지 7 -> 성공 종료 0~4%).
    # hold_gated_success=True 면 "유지 조건 달성" 시점에만 일시금을 준다.
    achieved_hold = (
        state.info["success_hold_steps"] >= self._config.success_hold_steps
    )
    if self._config.hold_gated_success:
      n_stairs = int(self._config.hold_stairs)
      if n_stairs > 1:
        # 에피소드 최고 기록이 새로 넘긴 계단 수만큼 지급한다. 최고 기록 기준이라
        # 계단당 최대 한 번만 나가고(반복 수령 불가), 총액은 게이트와 같다.
        step_size = max(self._config.success_hold_steps // n_stairs, 1)
        cleared = lambda b: jp.minimum(b // step_size, n_stairs)
        new_stairs = cleared(state.info["best_hold_steps"]) - cleared(prev_best)
        reward += (
            new_stairs * self._config.reward_config.success_reward / n_stairs
        )
      else:
        reward += achieved_hold * self._config.reward_config.success_reward
      state.metrics["reward/success"] = achieved_hold.astype(float)
    else:
      reward += success * self._config.reward_config.success_reward
      # hold_bonus > 0 이면 스텝당 성공 보상(조밀 신호)은 그대로 두고 유지 달성에
      # 일시금을 얹는다. 게이트 방식(hold_gated_success)은 착취는 막지만 조밀
      # 신호까지 없애서 SAC 에는 해로웠다(v31: 36.8% -> 27.8%). PPO 처럼 착취가
      # 가능한 정책에는 게이트가, SAC 처럼 신호가 필요한 정책에는 이 방식이 맞다.
      reward += achieved_hold * self._config.reward_config.hold_bonus

    # Update info and metrics.
    state.info["step"] += 1
    state.info["last_last_act"] = state.info["last_act"]
    state.info["last_act"] = action
    state.info["prev_ori_error"] = ori_error
    state.info["best_ori_error"] = jp.minimum(
        state.info["best_ori_error"], ori_error
    )

    # 보상은 여기까지 **옛 목표 기준**으로 계산됐다. 이제 목표를 바꾼다.
    if self._config.chain_goals:
      data = self._chain_next_goal(data, state.info, achieved_hold)

    obs = self._get_obs(data, state.info)
    # D7: 종료 사유. stuck 을 steps_since_goal 로 다시 판정하면 _chain_next_goal 의
    # 증가 때문에 done 계산 시점과 1스텝 어긋난다. done 에서 낙하를 빼는 방식이
    # 정의상 정확하다(done = 낙하 | 정체 | nan, nan 은 사실상 없음).
    dropped = self._cube_dropped(data)
    state.metrics["term_dropped"] = (done & dropped).astype(float)
    state.metrics["term_stuck"] = (done & ~dropped).astype(float)
    state.metrics["goal_count"] = state.info["goal_count"].astype(float)
    state.metrics["steps_since_goal"] = state.info["steps_since_goal"].astype(
        float
    )
    for k, v in rewards.items():
      state.metrics[f"reward/{k}"] = v

    done = done.astype(reward.dtype)
    state = state.replace(data=data, obs=obs, reward=reward, done=done)
    return state

  def _chain_next_goal(self, data: mjx.Data, info: dict[str, Any],
                       achieved: jax.Array) -> mjx.Data:
    """연쇄 모드의 목표 교체. 과제 2 가 구현한다(과제 1 은 그대로 둔다)."""
    del achieved
    info["steps_since_goal"] = info["steps_since_goal"] + 1
    return data

  def _cube_dropped(self, data: mjx.Data) -> jax.Array:
    """큐브를 떨어뜨렸는가. termination 보상항의 대상이다(D8)."""
    return self.get_cube_position(data)[2] < -0.05

  def _get_termination(self, data: mjx.Data, info: dict[str, Any]) -> jax.Array:
    fall_termination = self._cube_dropped(data)
    nans = jp.any(jp.isnan(data.qpos)) | jp.any(jp.isnan(data.qvel))
    if self._config.chain_goals:
      # 성공은 더 이상 종료 사유가 아니다(새 목표로 이어간다). 대신 한 목표에
      # 너무 오래 매달리면 정체로 보고 끝낸다.
      stuck = info["steps_since_goal"] >= self._config.goal_timeout_steps
      return fall_termination | stuck | nans
    success_termination = (
        info["success_hold_steps"] >= self._config.success_hold_steps
    )
    return fall_termination | success_termination | nans

  def _get_obs(
      self, data: mjx.Data, info: dict[str, Any]
  ) -> mjx_env.Observation:
    # Hand joint angles.
    joint_angles = data.qpos[self._hand_qids]
    info["rng"], noise_rng = jax.random.split(info["rng"])
    noisy_joint_angles = (
        joint_angles
        + (2 * jax.random.uniform(noise_rng, shape=joint_angles.shape) - 1)
        * self._config.obs_noise.level
        * self._config.obs_noise.scales.joint_pos
    )

    # Joint position error history.
    qpos_error_history = (
        jp.roll(info["qpos_error_history"], 16)
        .at[:16]
        .set(noisy_joint_angles - info["motor_targets"])
    )
    info["qpos_error_history"] = qpos_error_history

    def _get_cube_pose(data: mjx.Data) -> jax.Array:
      """Returns (potentially) noisy cube pose (xyz,wxyz)."""
      cube_pos = self.get_cube_position(data)
      cube_quat = self.get_cube_orientation(data)
      info["rng"], pos_rng, ori_rng = jax.random.split(info["rng"], 3)
      noisy_cube_quat = mjx._src.math.normalize(
          cube_quat
          + jax.random.normal(ori_rng, shape=(4,))
          * self._config.obs_noise.level
          * self._config.obs_noise.scales.cube_ori
      )
      noisy_cube_pos = (
          cube_pos
          + (2 * jax.random.uniform(pos_rng, shape=cube_pos.shape) - 1)
          * self._config.obs_noise.level
          * self._config.obs_noise.scales.cube_pos
      )
      return jp.concatenate([noisy_cube_pos, noisy_cube_quat])

    # Noisy cube pose.
    noisy_pose = _get_cube_pose(data)
    info["rng"], key1, key2, key3 = jax.random.split(info["rng"], 4)
    rand_quat = leap_hand_base.uniform_quat(key1)
    rand_pos = jax.random.uniform(key2, (3,), minval=-0.5, maxval=0.5)
    rand_pose = jp.concatenate([rand_pos, rand_quat])
    m = self._config.obs_noise.level * jax.random.bernoulli(
        key3, self._config.obs_noise.random_ori_injection_prob
    )
    noisy_pose = noisy_pose * (1 - m) + rand_pose * m

    # Cube position error history.
    palm_pos = self.get_palm_position(data)
    cube_pos_error = palm_pos - noisy_pose[:3]
    cube_pos_error_history = (
        jp.roll(info["cube_pos_error_history"], 3).at[:3].set(cube_pos_error)
    )
    info["cube_pos_error_history"] = cube_pos_error_history

    # Cube orientation error history.
    goal_quat = self._goal_quat(data)  # 센서 아님 (D2)
    quat_diff = mjx._src.math.quat_mul(
        noisy_pose[3:], mjx._src.math.quat_inv(goal_quat)
    )
    xmat_diff = mjx._src.math.quat_to_mat(quat_diff).ravel()[3:]
    cube_ori_error_history = (
        jp.roll(info["cube_ori_error_history"], 6).at[:6].set(xmat_diff)
    )
    info["cube_ori_error_history"] = cube_ori_error_history

    # Uncorrupted cube pose for critic.
    cube_pos_error_uncorrupted = palm_pos - self.get_cube_position(data)
    cube_quat_uncorrupted = self.get_cube_orientation(data)
    quat_diff_uncorrupted = math.quat_mul(
        cube_quat_uncorrupted, math.quat_inv(goal_quat)
    )
    xmat_diff_uncorrupted = math.quat_to_mat(quat_diff_uncorrupted).ravel()[3:]

    # scalar orientation error: policy가 현재 오차를 직접 인식하도록
    noisy_quat_diff = mjx._src.math.quat_mul(
        noisy_pose[3:], mjx._src.math.quat_inv(goal_quat)
    )
    noisy_quat_diff = mjx._src.math.normalize(noisy_quat_diff)
    noisy_ori_error = 2.0 * jp.asin(
        jp.clip(jp.linalg.norm(noisy_quat_diff[1:]), max=1.0)
    ).reshape(1)

    state = jp.concatenate([
        noisy_joint_angles,  # 16
        qpos_error_history,  # 16 * history_len
        cube_pos_error_history,  # 3 * history_len
        cube_ori_error_history,  # 6 * history_len
        info["last_act"],  # 16
        noisy_ori_error,  # 1
    ])

    privileged_state = jp.concatenate([
        state,
        data.qpos[self._hand_qids],
        data.qvel[self._hand_dqids],
        self.get_fingertip_positions(data),
        cube_pos_error_uncorrupted,
        xmat_diff_uncorrupted,
        self.get_cube_linvel(data),
        self.get_cube_angvel(data),
        info["pert_dir"],
        data.xfrc_applied[self._cube_body_id],
    ])

    return {
        "state": state,
        "privileged_state": privileged_state,
    }

  # Reward terms.

  def _get_reward(
      self,
      data: mjx.Data,
      action: jax.Array,
      info: dict[str, Any],
      metrics: dict[str, Any],
      done: jax.Array,
  ) -> dict[str, jax.Array]:
    del done, metrics  # Unused.

    cube_pos = self.get_cube_position(data)
    palm_pos = self.get_palm_position(data)
    cube_pose_mse = jp.linalg.norm(palm_pos - cube_pos)
    cube_pos_reward = reward.tolerance(
        cube_pose_mse, (0, 0.02), margin=0.05, sigmoid="linear"
    )

    # D8: termination(scale -200)의 대상은 **큐브를 떨어뜨림**이다.
    # _get_termination 을 그대로 쓰면 연쇄 모드의 정체 종료(오래 걸림)에도
    # 같은 벌점이 붙고, 반대로 성공이 종료가 아니게 되면서 비연쇄에서 성공 시
    # 함께 물던 -200*dt 가 사라진다. 두 방향 모두 기준선과 보상 구성이 달라진다.
    # 낙하만 보도록 고정해 **연쇄 여부와 무관하게 같은 의미**로 만든다.
    terminated = self._cube_dropped(data)

    hand_pose_reward = jp.sum(
        jp.square(data.qpos[self._hand_qids] - self._default_pose)
    )

    # Progress reward: 개선은 +, 악화는 -로 주어 보상 해킹을 줄인다.
    cur_ori_error = self._cube_orientation_error(data, info)
    delta_progress = jp.clip(info["prev_ori_error"] - cur_ori_error, -0.3, 0.3)
    best_progress = jp.clip(info["best_ori_error"] - cur_ori_error, 0.0, 0.5)

    return {
        "orientation": self._reward_cube_orientation(data, info),
        "progress": delta_progress,
        "best_progress": best_progress,
        "direct_penalty": -cur_ori_error,
        "precision": jp.exp(-4.0 * cur_ori_error),
        "fine_025": jp.clip((0.40 - cur_ori_error) / 0.15, 0.0, 1.0),
        "fine_020": jp.clip((0.30 - cur_ori_error) / 0.10, 0.0, 1.0),
        "position": cube_pos_reward,
        "termination": terminated,
        "hand_pose": hand_pose_reward,
        "action_rate": self._cost_action_rate(
            action, info["last_act"], info["last_last_act"]
        ),
        "joint_vel": self._cost_joint_vel(data),
        "joint_limit": self._cost_joint_limit(data),
        "energy": self._cost_energy(
            data.qvel[self._hand_dqids], data.actuator_force
        ),
    }

  def _cost_energy(
      self, qvel: jax.Array, qfrc_actuator: jax.Array
  ) -> jax.Array:
    return jp.sum(jp.abs(qvel) * jp.abs(qfrc_actuator))

  def _goal_relative_xmat(self, data: mjx.Data,
                          cube_quat: jax.Array = None) -> jax.Array:
    """관측 이력(`cube_ori_error_history`)에 들어가는 6원소 값.

    `_get_obs` 가 쓰는 식과 **같은 식**이어야 한다. 연쇄 모드에서 목표를 바꾼 뒤
    이력을 새 목표 기준으로 채울 때 재사용한다(roll 이 아니라 fill).
    """
    q = self.get_cube_orientation(data) if cube_quat is None else cube_quat
    qd = math.quat_mul(q, math.quat_inv(self._goal_quat(data)))
    return math.quat_to_mat(qd).ravel()[3:]

  def _goal_quat(self, data: mjx.Data) -> jax.Array:
    """목표 사원수를 **센서가 아니라 `mocap_quat` 에서 직접** 읽는다.

    `get_cube_goal_orientation` 은 framequat 센서다. `data.replace(mocap_quat=...)`
    는 센서를 갱신하지 않으므로(물리 forward 전) 목표를 바꾼 직후 한 스텝 동안
    **옛 목표**를 돌려준다. 실측:
        reset 직후    mocap [0.707,0,0.707,0]  센서 [0,0.707,0,0.707]  face_err 2.87
        물리 1스텝 뒤 mocap [0.707,0,0.707,0]  센서 [0.707,0,0.707,0]  face_err 0.27
    연쇄 모드는 매 목표 교체마다 이 창을 만나므로 치명적이다(best_ori_error 가
    옛 목표의 ~0 으로 박혀 best_progress(scale 80)가 에피소드 내내 죽는다).
    `_face_index` 가 이미 같은 이유로 mocap 을 직접 읽고 있었다.

    mocap body 는 `goal` 하나뿐이다(nmocap=1).
    """
    return data.mocap_quat.reshape(-1)[:4]

  def _cube_orientation_error(self, data: mjx.Data, info=None):
    """info 는 하위 클래스(과제2)가 에피소드별 기준점을 쓰기 위한 통로다.

    과제 1 은 쓰지 않으므로 무시한다. 기본값 None 이면 기존 동작 그대로다.
    """
    del info
    cube_ori = self.get_cube_orientation(data)
    cube_goal_ori = self._goal_quat(data)  # 센서 아님 (D2)
    quat_diff = math.quat_mul(cube_ori, math.quat_inv(cube_goal_ori))
    quat_diff = math.normalize(quat_diff)
    return 2.0 * jp.asin(jp.clip(math.norm(quat_diff[1:]), max=1.0))

  def _reward_cube_orientation(self, data: mjx.Data, info=None) -> jax.Array:
    ori_error = self._cube_orientation_error(data, info)
    # 선형항: error=0→1.0, error=π→0.0, 전 범위에서 상수 gradient 보장
    linear = (jp.pi - ori_error) / jp.pi
    # exp(-0.5*x): error=2.79→0.25, 큰 오차에서도 gradient 잔존
    exp_shaped = jp.exp(-0.5 * ori_error)
    # near_goal: success_threshold 기반 동적 범위
    # margin = thr*4 → stage1(0.60): shaping zone 0.60~3.0 rad (현재 error 포함)
    #                  stage6(0.10): shaping zone 0.10~0.50 rad
    thr = self._config.success_threshold
    margin = jp.clip(thr * 4.0, 0.5, jp.pi - thr)
    near_goal_enabled = jp.asarray(thr <= 1.0, dtype=float)
    near_goal = near_goal_enabled * jp.where(
        ori_error <= thr,
        1.0,
        jp.clip(1.0 - (ori_error - thr) / margin, 0.0, 1.0),
    )
    success_band_bonus = (
        near_goal_enabled * (ori_error < thr).astype(float) * 2.0
    )
    coarse_bonus_enabled = jp.asarray(thr <= 0.8, dtype=float)
    coarse_align_bonus = (
        coarse_bonus_enabled * (ori_error < 0.6).astype(float) * 4.0
    )
    return linear + exp_shaped + near_goal + success_band_bonus + coarse_align_bonus

  def _cost_action_rate(
      self, act: jax.Array, last_act: jax.Array, last_last_act: jax.Array
  ) -> jax.Array:
    c1 = jp.sum(jp.square(act - last_act))
    c2 = jp.sum(jp.square(act - 2 * last_act + last_last_act))
    return c1 + c2

  def _cost_joint_limit(self, data: mjx.Data) -> jax.Array:
    """가동 범위 바깥 `soft_factor` 밖에 들어간 정도를 잰다.

    SPiDR(arXiv:2509.18648)의 soft factor 0.75 규약을 따른다: 관절이 범위의
    바깥 25% 에 들어가면 비용이 붙고, 한계에 가까울수록 선형으로 커진다.
    하드 클립(이미 걸려 있다)은 그 각도에 못 가게 막을 뿐 정책이 한계를 덜
    **원하도록** 만들지는 못한다. 이 항이 그 역할을 한다.
    """
    q = data.qpos[self._hand_qids]
    center = 0.5 * (self._lowers + self._uppers)
    half = 0.5 * (self._uppers - self._lowers)
    # 중심에서의 정규화 거리: 0 = 중앙, 1 = 한계
    dist = jp.abs(q - center) / jp.maximum(half, 1e-6)
    soft = self._config.joint_limit_soft_factor
    return jp.sum(jp.clip(dist - soft, 0.0, None)) / max(1.0 - soft, 1e-6)

  def _cost_joint_vel(self, data: mjx.Data) -> jax.Array:
    max_velocity = 5.0
    vel_tolerance = 1.0
    hand_qvel = data.qvel[self._hand_dqids]
    return jp.sum((hand_qvel / (max_velocity - vel_tolerance)) ** 2)

  # Perturbation.

  def _maybe_apply_perturbation(
      self, state: mjx_env.State, rng: jax.Array
  ) -> mjx_env.State:
    def gen_dir(rng: jax.Array) -> jax.Array:
      directory = jax.random.normal(rng, (6,))
      return directory / jp.linalg.norm(directory)

    def get_xfrc(
        state: mjx_env.State, pert_dir: jax.Array, i: jax.Array
    ) -> jax.Array:
      u_t = 0.5 * jp.sin(jp.pi * i / state.info["pert_duration_steps"])
      force = (
          u_t
          * self._cube_mass
          * state.info["pert_vel"]
          / (state.info["pert_duration_steps"] * self.dt)
      )
      xfrc_applied = jp.zeros((self.mjx_model.nbody, 6))
      xfrc_applied = xfrc_applied.at[self._cube_body_id].set(force * pert_dir)
      return xfrc_applied

    step, last_pert_step = state.info["step"], state.info["last_pert_step"]
    start_pert = jp.mod(step, state.info["pert_wait_steps"]) == 0
    start_pert &= step != 0  # No perturbation at the beginning of the episode.
    last_pert_step = jp.where(start_pert, step, last_pert_step)
    duration = jp.clip(step - last_pert_step, 0, 100_000)
    in_pert_interval = duration < state.info["pert_duration_steps"]

    pert_dir = jp.where(start_pert, gen_dir(rng), state.info["pert_dir"])
    xfrc = get_xfrc(state, pert_dir, duration) * in_pert_interval

    state.info["pert_dir"] = pert_dir
    state.info["last_pert_step"] = last_pert_step
    data = state.data.replace(xfrc_applied=xfrc)
    return state.replace(data=data)


def domain_randomize(model: mjx.Model, rng: jax.Array):
  # geom/body id 조회용으로만 환경을 하나 세운다. 설정 없이 만들면 기본
  # impl='warp' 가 걸려 이 환경(warp 미지원)에서 mjx.put_model 이 죽는다.
  # id 조회는 impl 과 무관하므로 jax 로 고정한다.
  _cfg = default_config()
  _cfg.impl = "jax"
  mj_model = CubeReorient(config=_cfg).mj_model
  cube_geom_id = mj_model.geom("cube").id
  cube_body_id = mj_model.body("cube").id
  hand_qids = mjx_env.get_qpos_ids(mj_model, consts.JOINT_NAMES)
  hand_body_names = [
      "palm",
      "if_bs",
      "if_px",
      "if_md",
      "if_ds",
      "mf_bs",
      "mf_px",
      "mf_md",
      "mf_ds",
      "rf_bs",
      "rf_px",
      "rf_md",
      "rf_ds",
      "th_mp",
      "th_bs",
      "th_px",
      "th_ds",
  ]
  hand_body_ids = np.array([mj_model.body(n).id for n in hand_body_names])
  fingertip_geoms = ["th_tip", "if_tip", "mf_tip", "rf_tip"]
  fingertip_geom_ids = [mj_model.geom(g).id for g in fingertip_geoms]

  @jax.vmap
  def rand(rng):
    rng, key = jax.random.split(rng)
    # Fingertip friction: =U(0.5, 1.0).
    fingertip_friction = jax.random.uniform(key, (1,), minval=0.5, maxval=1.0)
    geom_friction = model.geom_friction.at[fingertip_geom_ids, 0].set(
        fingertip_friction
    )

    # Scale cube mass: *U(0.8, 1.2).
    rng, key1, key2 = jax.random.split(rng, 3)
    dmass = jax.random.uniform(key1, minval=0.8, maxval=1.2)
    body_inertia = model.body_inertia.at[cube_body_id].set(
        model.body_inertia[cube_body_id] * dmass
    )
    dpos = jax.random.uniform(key2, (3,), minval=-5e-3, maxval=5e-3)
    body_ipos = model.body_ipos.at[cube_body_id].set(
        model.body_ipos[cube_body_id] + dpos
    )

    # Jitter qpos0: +U(-0.05, 0.05).
    rng, key = jax.random.split(rng)
    qpos0 = model.qpos0
    qpos0 = qpos0.at[hand_qids].set(
        qpos0[hand_qids]
        + jax.random.uniform(key, shape=(16,), minval=-0.05, maxval=0.05)
    )

    # Scale static friction: *U(0.9, 1.1).
    rng, key = jax.random.split(rng)
    frictionloss = model.dof_frictionloss[hand_qids] * jax.random.uniform(
        key, shape=(16,), minval=0.5, maxval=2.0
    )
    dof_frictionloss = model.dof_frictionloss.at[hand_qids].set(frictionloss)

    # Scale armature: *U(1.0, 1.05).
    rng, key = jax.random.split(rng)
    armature = model.dof_armature[hand_qids] * jax.random.uniform(
        key, shape=(16,), minval=1.0, maxval=1.05
    )
    dof_armature = model.dof_armature.at[hand_qids].set(armature)

    # Scale all link masses: *U(0.9, 1.1).
    rng, key = jax.random.split(rng)
    dmass = jax.random.uniform(
        key, shape=(len(hand_body_ids),), minval=0.9, maxval=1.1
    )
    body_mass = model.body_mass.at[hand_body_ids].set(
        model.body_mass[hand_body_ids] * dmass
    )

    # Joint stiffness: *U(0.8, 1.2).
    rng, key = jax.random.split(rng)
    kp = model.actuator_gainprm[:, 0] * jax.random.uniform(
        key, (model.nu,), minval=0.8, maxval=1.2
    )
    actuator_gainprm = model.actuator_gainprm.at[:, 0].set(kp)
    actuator_biasprm = model.actuator_biasprm.at[:, 1].set(-kp)

    # Joint damping: *U(0.8, 1.2).
    rng, key = jax.random.split(rng)
    kd = model.dof_damping[hand_qids] * jax.random.uniform(
        key, (16,), minval=0.8, maxval=1.2
    )
    dof_damping = model.dof_damping.at[hand_qids].set(kd)

    # 토크 제한: *U(0.8, 1.9).
    # 원본 목록에는 없던 항목이다. 실측 결과 정책이 학습 토크에만 뾰족하게
    # 맞춰진다(0.11 에서 85~89%, 0.21 로 올리면 14~20%). 실기는 전류 설정에
    # 따라 토크가 달라지므로 이 축을 덮어야 한다. 0.11 기준으로 [0.088, 0.209]
    # 이고, 참조 구현 값 0.21 까지 포함한다. 아래로는 0.05 에서 두 정책 모두
    # 0% 였으므로 실행 불가 구간을 피해 0.8 배까지만 내린다.
    rng, key = jax.random.split(rng)
    force_scale = jax.random.uniform(key, minval=0.8, maxval=1.9)
    actuator_forcerange = model.actuator_forcerange * force_scale

    return (
        geom_friction,
        body_mass,
        body_inertia,
        body_ipos,
        qpos0,
        dof_frictionloss,
        dof_armature,
        dof_damping,
        actuator_gainprm,
        actuator_biasprm,
        actuator_forcerange,
    )

  (
      geom_friction,
      body_mass,
      body_inertia,
      body_ipos,
      qpos0,
      dof_frictionloss,
      dof_armature,
      dof_damping,
      actuator_gainprm,
      actuator_biasprm,
      actuator_forcerange,
  ) = rand(rng)

  in_axes = jax.tree_util.tree_map(lambda x: None, model)
  in_axes = in_axes.tree_replace({
      "geom_friction": 0,
      "body_mass": 0,
      "body_inertia": 0,
      "body_ipos": 0,
      "qpos0": 0,
      "dof_frictionloss": 0,
      "dof_armature": 0,
      "dof_damping": 0,
      "actuator_gainprm": 0,
      "actuator_biasprm": 0,
      "actuator_forcerange": 0,
  })

  model = model.tree_replace({
      "geom_friction": geom_friction,
      "body_mass": body_mass,
      "body_inertia": body_inertia,
      "body_ipos": body_ipos,
      "qpos0": qpos0,
      "dof_frictionloss": dof_frictionloss,
      "dof_armature": dof_armature,
      "dof_damping": dof_damping,
      "actuator_gainprm": actuator_gainprm,
      "actuator_biasprm": actuator_biasprm,
      "actuator_forcerange": actuator_forcerange,
  })

  return model, in_axes
