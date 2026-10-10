"""교사 위에 residual 을 얹는다. 학생은 교정량만 배운다.

    a = clip(a_교사(s) + scale · a_residual(s), -1, 1)

왜 이 구조인가 (2026-08-27 핸드오프 진단)
  교사가 25스텝만 굴려주면 학생 성공률이 80.8 -> 93.6% 로 뛰고, 200스텝이면
  96.3% 로 교사(99.1%)에 근접한다. 반대로 학생이 만든 상태는 교사가 이어받으면
  93.8% 로 대체로 회복된다. 즉 학생의 표현력도, 학생이 만든 상태의 회복
  가능성도 문제가 아니다. **학생이 자기 궤적 위에서만 못 한다** — 복합 오차다.

  residual 구조는 이 문제를 학습이 아니라 **구조로** 없앤다. 자기 궤적 위에서도
  base 가 교사 행동을 내므로 분포 이탈이 원리적으로 생기지 않는다.
  (ResFiT, arXiv:2509.19301 — 29-DoF 실기에서 14%->64%)

설계 판단 셋

  1. 관측에 a_교사 를 넣지 않는다.
     ResFiT 은 π_res(s, a_base) 를 쓰지만, 우리 교사는 **결정적**이라
     a_교사 = f(s) 이고 s 만으로 복원 가능하다. 관측 차원을 108 로 유지하면
     기존 평가·진단 도구가 그대로 돌아가고 비교도 깨지지 않는다.

  2. BC 를 쓰지 않는다.
     residual=0 이 이미 교사 성능(99.6%)이다. 시연의 residual 은 정의상 0 이라
     BC 를 걸면 "교사를 그대로 따라 하라"가 되어 학습을 막는다.
     v24 에서 BC 를 끄면 무너졌던 이유(교사 분포로 당기는 유일한 힘이 사라짐)가
     여기서는 사라진다 — base 자체가 교사다.

  3. wrapper 로 만든다.
     SAC 학습 코드를 건드리지 않는다. 학생은 평범한 108차원 관측·16차원 행동
     문제를 풀고, 행동의 의미만 "교정량"으로 바뀐다.

배치 주의
  이 wrapper 는 **raw env 바로 위, ObsKeyWrapper 아래**에 놓아야 한다.
  교사(PPO)는 dict 관측을 받으므로 평탄화 전에 호출해야 한다.
"""

from typing import Any, Callable

import jax
import jax.numpy as jp

from mujoco_playground._src import mjx_env
from mujoco_playground._src import wrapper as pg_wrapper


class ResidualTeacherWrapper(pg_wrapper.Wrapper):
  """학생 행동을 교사 행동에 대한 교정량으로 해석한다."""

  def __init__(self, env: Any, teacher_fn: Callable, scale: float = 0.3,
               l2_cost: float = 0.0):
    super().__init__(env)
    self._teacher = teacher_fn
    self._scale = float(scale)
    # residual 을 0 근처로 당기는 비용. v35 에서 이 힘이 없자 SAC 이 엔트로피
    # 탐색으로 residual 을 넓게 훑어 교사를 훼손했다(99.6% -> 51.6%,
    # scale=1.0 에서는 0.0%). 시연의 residual 은 정의상 0 이라 BC 로는 못 건다.
    self._l2 = float(l2_cost)
    # 교사는 결정적이므로 rng 는 형식상 필요할 뿐이다. 고정 키를 쓴다.
    self._key = jax.random.PRNGKey(0)

  def _combine(self, obs, a_res):
    a_teacher, _ = self._teacher(obs, self._key)
    return jp.clip(a_teacher + self._scale * a_res, -1.0, 1.0)

  def reset(self, rng: jax.Array) -> mjx_env.State:
    return self.env.reset(rng)

  def step(self, state: mjx_env.State, action: jax.Array) -> mjx_env.State:
    nstate = self.env.step(state, self._combine(state.obs, action))
    if self._l2 > 0.0:
      # 스텝당 보상에서 뺀다. 성공 일시금(2000)에 비하면 작지만, 유지 구간의
      # 스텝당 보상(약 10)에 대해서는 유효한 압력이 된다.
      nstate = nstate.replace(
          reward=nstate.reward - self._l2 * jp.sum(jp.square(action), axis=-1))
    return nstate
