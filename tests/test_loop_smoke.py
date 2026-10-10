"""제어 루프 구성요소를 실기 없이 굴려 본다.

ObsBuilder + Policy + limits 를 실제로 조합해 100스텝 돌리고,
관측 차원·유한성·명령 범위·행동 크기를 확인한다.
"""
import sys, pathlib, time
D = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(D / "deploy"))
import numpy as np
import limits, obs_builder as ob, policy_np

pol = policy_np.Policy(str(D / "policy" / "policy_np.npz"))
b = ob.ObsBuilder()
goal = ob.FACE_GOAL_QUATS[0]
q_cmd = limits.DEFAULT_POSE.copy()
rng = np.random.default_rng(0)

worst_obs, t0 = 0.0, time.time()
for k in range(100):
    # 서보가 명령을 조금 늦게 따라온다고 가정
    q_meas = q_cmd + rng.normal(0, 0.01, 16)
    # 손바닥 기준 큐브: 손바닥 위 약 4cm, 자세는 천천히 회전
    palm = np.array([0.0, 0.0, 0.04]) + rng.normal(0, 0.002, 3)
    th = k * 0.02
    cube_quat = np.array([np.cos(th / 2), 0, np.sin(th / 2), 0])
    obs = b.build(q_meas, q_cmd, palm, np.zeros(3), cube_quat, goal)
    assert obs.shape == (108,), obs.shape
    assert np.all(np.isfinite(obs)), "관측에 비유한값"
    worst_obs = max(worst_obs, float(np.abs(obs).max()))
    a = pol(obs)
    assert a.shape == (16,) and np.all(np.abs(a) <= 1.0 + 1e-9), a
    b.set_last_act(a)
    q_cmd = np.clip(q_cmd + limits.ACTION_SCALE * a,
                    limits.SAFE_LOWERS, limits.SAFE_UPPERS)
    assert np.all(q_cmd >= limits.SAFE_LOWERS - 1e-9)
    assert np.all(q_cmd <= limits.SAFE_UPPERS + 1e-9)

el = (time.time() - t0) / 100 * 1000
print(f"100스텝 통과. 관측 최대 절대값 {worst_obs:.2f}")
print(f"관측조립+추론 {el:.2f} ms/스텝  (예산 {limits.CTRL_DT*1000:.0f} ms)")
print(f"명령이 안전범위 안에 유지됨. 최종 q_cmd 범위 "
      f"[{q_cmd.min():.3f}, {q_cmd.max():.3f}]")
