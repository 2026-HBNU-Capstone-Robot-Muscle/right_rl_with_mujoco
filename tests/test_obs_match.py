"""배포용 관측 조립기가 시뮬 env 의 _get_obs 와 같은 값을 내는지 확인한다.

관측 잡음을 끄고(obs_noise.level=0) 비교하므로 결정적이다. 여기서 어긋나면
실기에서 정책이 오류 없이 엉뚱하게 움직인다.
"""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "deploy"))

import jax, numpy as np
from brax.training import networks as bn
bn.KERNEL_INITIALIZER.setdefault(None, None)
from mujoco_playground import registry
from mujoco_playground._src.manipulation.leap_hand import leap_hand_constants as consts
import obs_builder as ob

OV = {"impl": "jax", "success_threshold": 0.15, "position_threshold": 0.015,
      "success_hold_steps": 20, "hold_gated_success": True, "settle_steps": 8,
      "chain_goals": True, "goal_timeout_steps": 600, "hyperext_floor": -0.2,
      "actuator_force_limit": 0.11, "action_scale": 0.2, "target_face": -1}
OV["obs_noise"] = {"level": 0.0, "scales": {"joint_pos": 0.05, "cube_pos": 0.02, "cube_ori": 0.1}, "random_ori_injection_prob": 0.0}

env = registry.load("LeapCubeFaceUp", config_overrides=OV)
reset = jax.jit(env.reset); step = jax.jit(env.step)
st = reset(jax.random.PRNGKey(0))

qids = [env.mj_model.joint(n).qposadr[0] for n in consts.JOINT_NAMES]
b = ob.ObsBuilder()
# 시뮬의 이력 버퍼를 그대로 이식한다(리셋 직후 상태를 맞추기 위해)
b.qpos_err_hist = np.asarray(st.info["qpos_error_history"], float).copy()
b.cube_pos_err_hist = np.asarray(st.info["cube_pos_error_history"], float).copy()
b.cube_ori_err_hist = np.asarray(st.info["cube_ori_error_history"], float).copy()
b.last_act = np.asarray(st.info["last_act"], float).copy()

key = jax.random.PRNGKey(1)
worst = 0.0
for t in range(12):
    key, k = jax.random.split(key)
    a = jax.random.uniform(k, (env.action_size,), minval=-1, maxval=1)
    st = step(st, a)
    d = st.data
    q_meas = np.asarray(d.qpos[np.array(qids)], float)
    q_cmd = np.asarray(st.info["motor_targets"], float)
    palm = np.asarray(env.get_palm_position(d), float)
    cpos = np.asarray(env.get_cube_position(d), float)
    cquat = np.asarray(env.get_cube_orientation(d), float)
    gquat = np.asarray(d.mocap_quat.reshape(-1)[:4], float)
    b.set_last_act(np.asarray(st.info["last_act"], float))
    mine = b.build(q_meas, q_cmd, palm, cpos, cquat, gquat)
    theirs = np.asarray(st.obs["state"], float)
    e = float(np.abs(mine - theirs).max())
    worst = max(worst, e)
    if t < 3 or e > 1e-5:
        print(f"  t={t:>2}  최대 절대오차 {e:.3e}")
assert mine.shape == (108,), mine.shape
print(f"\n차원 {mine.shape[0]}  12스텝 최대 오차 {worst:.3e}")
print("결과:", "일치" if worst < 1e-5 else "불일치 - 조립기를 고쳐야 한다")
