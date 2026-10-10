"""카메라 프레임 -> 월드 프레임 변환이 맞는지 합성으로 검증한다.

월드에서 손과 큐브의 자세를 정해 놓고 임의의 카메라로 관측한 뒤,
track_cube 의 변환이 원래 월드 값을 복원하는지 본다. 카메라를 어디에 놓든
같은 답이 나와야 한다 - 그것이 이 변환의 존재 이유다.
"""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "vision" / "rubik_center"))
import numpy as np
import track_cube as tc

rng = np.random.default_rng(0)


def rand_R():
    q = rng.normal(size=4); q /= np.linalg.norm(q)
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


tc.PALM_IN_MARKER = np.array([0.01, -0.02, 0.03])
tc.HAND_TO_WORLD = np.eye(3)

worst_d, worst_R = 0.0, 0.0
for trial in range(200):
    # 월드에서의 진값
    R_hand_w = rand_R()                       # 손 마커 자세 (월드)
    t_hand_w = rng.normal(scale=0.1, size=3)
    R_cube_w = rand_R()
    t_cube_w = t_hand_w + rng.normal(scale=0.05, size=3)
    palm_w = t_hand_w + R_hand_w @ tc.PALM_IN_MARKER
    truth_d = palm_w - t_cube_w               # 관측이 원하는 값
    # 임의의 카메라
    R_cw = rand_R(); t_cw = rng.normal(scale=0.5, size=3)   # 월드 -> 카메라
    R_hand_c = R_cw @ R_hand_w;  t_hand_c = R_cw @ t_hand_w + t_cw
    R_cube_c = R_cw @ R_cube_w;  t_cube_c = R_cw @ t_cube_w + t_cw
    # track_cube 가 하는 계산
    R_wc = tc.HAND_TO_WORLD @ R_hand_c.T
    palm_cam = t_hand_c + R_hand_c @ tc.PALM_IN_MARKER
    got_d = R_wc @ (palm_cam - t_cube_c)
    got_R = R_wc @ R_cube_c
    # 월드가 손 마커 프레임으로 정의되므로 진값도 같은 기준으로 옮긴다
    exp_d = R_hand_w.T @ truth_d
    exp_R = R_hand_w.T @ R_cube_w
    worst_d = max(worst_d, np.abs(got_d - exp_d).max())
    worst_R = max(worst_R, np.abs(got_R - exp_R).max())

print(f"손바닥-큐브 벡터 최대 오차 {worst_d:.2e} m")
print(f"큐브 회전행렬 최대 오차   {worst_R:.2e}")
print("결과:", "통과 - 카메라 위치와 무관하게 같은 값" if max(worst_d, worst_R) < 1e-12
      else "실패")

# 같은 장면을 서로 다른 카메라 두 대로 봤을 때 일치하는가
R_hand_w, t_hand_w, R_cube_w = rand_R(), np.zeros(3), rand_R()
t_cube_w = np.array([0.02, -0.01, 0.05])
outs = []
for _ in range(2):
    R_cw, t_cw = rand_R(), rng.normal(scale=0.5, size=3)
    R_h, t_h = R_cw @ R_hand_w, R_cw @ t_hand_w + t_cw
    R_c, t_c = R_cw @ R_cube_w, R_cw @ t_cube_w + t_cw
    R_wc = tc.HAND_TO_WORLD @ R_h.T
    outs.append((R_wc @ (t_h + R_h @ tc.PALM_IN_MARKER - t_c), R_wc @ R_c))
print(f"\n두 카메라 간 불일치  위치 {np.abs(outs[0][0]-outs[1][0]).max():.2e} m, "
      f"자세 {np.abs(outs[0][1]-outs[1][1]).max():.2e}")
