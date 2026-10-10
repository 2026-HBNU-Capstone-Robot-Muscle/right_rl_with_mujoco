"""20 Hz 제어 루프. 시뮬의 env.step 앞부분을 그대로 옮긴 것이다.

    q_cmd = clip(q_cmd + 0.2 * a, SAFE_LOWERS, SAFE_UPPERS)

ema_alpha = 1.0 이라 명령 평활이 없다. **실기에서 저역통과 필터를 얹지 말 것.**
떨림은 정책의 성질이고, 필터로 덮으면 재파지 동작까지 함께 느려진다.

누적은 직전 **명령각**에 한다. 측정각에 누적하면 서보 추종 오차가 매 스텝
누적 경로에 섞여 전혀 다른 동역학이 된다.
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "vision" / "rubik_center"))

import limits
import leap_hw
import obs_builder as ob
import policy_np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", default="../policy/policy_np.npz")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--cams", nargs="*", default=[],
                    help="장치경로:camera.npz 쌍. 비우면 큐브 자세 없이 동작 확인만")
    ap.add_argument("--marker_mm", type=float, default=12.0)
    ap.add_argument("--face_rot", default="0,0,0,0,0,0")
    ap.add_argument("--face", type=int, default=-1,
                    help="첫 목표 면 0~5. -1 이면 무작위")
    ap.add_argument("--chain", action="store_true",
                    help="성공하면 리셋 없이 다음 목표를 뽑는다(시뮬 지표와 동일)")
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--curr_lim", type=int, default=leap_hw.CURR_LIM)
    ap.add_argument("--dry_run", action="store_true", help="서보에 쓰지 않는다")
    ap.add_argument("--max_stale", type=float, default=0.15,
                    help="이 시간(초)보다 오래된 자세면 명령을 얼린다")
    a = ap.parse_args()

    pol = policy_np.Policy(a.policy)
    builder = ob.ObsBuilder()
    rng = np.random.default_rng(0)
    face = a.face if a.face >= 0 else int(rng.integers(6))
    goal = ob.FACE_GOAL_QUATS[face]
    hold = 0            # 연속 유지 스텝
    goals = 0           # 연속 달성 횟수
    p0 = None           # 에피소드 시작 시 큐브 위치(손바닥 기준)

    if not a.cams:
        # 큐브 자세 없이 정책을 굴리면 관측의 26% 가 거짓값이 되고, 그 상태로
        # 서보에 쓰면 손이 예측 불가능하게 움직인다.
        raise SystemExit("--cams 가 필요하다. 서보 없이 확인만 하려면 --dry_run "
                         "과 함께 --cams 를 그대로 주거나, check_joints.py 를 쓸 것.")
    if True:
        import multi_camera
        pairs = [tuple(c.split(":")) for c in a.cams]
        cam = multi_camera.MultiCamCube(
            pairs, marker_mm=a.marker_mm,
            face_rot=tuple(int(x) for x in a.face_rot.split(",")))

    hw = None if a.dry_run else leap_hw.LeapHW(port=a.port, curr_lim=a.curr_lim)
    if hw:
        print(f"토크 상한 {hw.torque_nm():.4f} Nm (전류 {a.curr_lim} mA)")

    # 기본 자세로 이동한 뒤 settle 동안 정책 없이 대기
    q_cmd = limits.DEFAULT_POSE.copy()
    if hw:
        hw.move_to(q_cmd, seconds=2.0)      # 계단 명령 금지. 램프로 간다.
    time.sleep(0.5)
    print(f"안착 대기 {limits.SETTLE_STEPS} 스텝 "
          f"({limits.SETTLE_STEPS * limits.CTRL_DT:.2f}초) - 큐브를 손 위에 올려주세요")
    time.sleep(limits.SETTLE_STEPS * limits.CTRL_DT)

    dt = limits.CTRL_DT
    n = int(a.seconds / dt)
    lat = []
    no_pose = stale = frozen = over = 0
    t_next = time.time()
    for k in range(n):
        t0 = time.time()
        t_next += dt                      # 마감 시각은 경로와 무관하게 먼저 민다
        q_meas = hw.read_sim() if hw else q_cmd.copy()

        r = cam.read()
        hold = False
        if r is None:
            no_pose += 1                  # 아직 한 번도 못 봤다
            hold = True
        elif r["stale"]:
            stale += 1
            if r["age"] > a.max_stale:
                # 오래된 자세로 계속 굴리면 정책이 헛돈다. 명령을 얼린다.
                frozen += 1
                hold = True

        if not hold:
            # track_cube 가 이미 **월드 프레임**으로 돌려준 값이다.
            # 카메라 프레임 값을 그대로 넣으면 카메라를 어디 두느냐에 따라
            # 관측이 통째로 회전한다.
            palm = np.asarray(r["palm_minus_cube"], float)
            cube_quat = np.asarray(r["cube_quat_world"], float)
            if p0 is None:
                p0 = -palm.copy()          # 시작 큐브 위치. 제자리 복귀의 기준점
            obs = builder.build(q_meas, q_cmd, palm, np.zeros(3), cube_quat, goal)
            act = pol(obs)
            builder.set_last_act(act)
            q_cmd = np.clip(q_cmd + limits.ACTION_SCALE * act,
                            limits.SAFE_LOWERS, limits.SAFE_UPPERS)

            # --- 성공 판정. 시뮬의 _cube_orientation_error 와 같은 식이다 ---
            #   task_error = max(면 오차, 중심 오차 * thr/pos_thr)
            # max 형태라 둘 다 만족해야 성공이다(동시 만족).
            fe = _face_error(cube_quat, face)
            ce = float(np.linalg.norm(-palm - p0))
            te = max(fe, ce * limits.SUCCESS_THRESHOLD / limits.POSITION_THRESHOLD)
            hold = hold + 1 if te < limits.SUCCESS_THRESHOLD else 0
            if hold >= limits.HOLD_STEPS:
                goals += 1
                hold = 0
                print(f"  [{k:>4}] 목표 달성 {goals}회  면오차 {np.degrees(fe):.1f}도"
                      f"  중심오차 {ce*1000:.1f}mm")
                if not a.chain:
                    break
                # 지금 위를 향한 면은 제외하고 새 목표를 뽑는다(즉시 성공 방지)
                up_now = _face_up(cube_quat)
                cand = [i for i in range(6) if i != up_now]
                face = int(rng.choice(cand))
                goal = ob.FACE_GOAL_QUATS[face]
                builder.fill_ori_history(cube_quat, goal)   # 옛 목표 잔상 제거

        if hw:
            hw.write_sim(q_cmd)           # hold 면 같은 명령을 다시 보낸다
        lat.append(time.time() - t0)

        late = time.time() - t_next
        if late > 0:
            over += 1
            if late > dt:
                # 크게 밀렸으면 따라잡지 않는다. 밀린 만큼 몰아서 도는 것이
                # 실제 지연보다 나쁜 동작을 만든다.
                t_next = time.time()
        else:
            time.sleep(-late)

    lat = np.array(lat) * 1000
    print(f"\n스텝 {n}  루프 시간 평균 {lat.mean():.1f} ms  최대 {lat.max():.1f} ms"
          f"  (예산 {dt*1000:.0f} ms)")
    print(f"연속 달성 {goals}회  (시뮬 기준선: 교사 팔F 0.11Nm 에서 7.19)")
    print(f"큐브 자세  미검출 {no_pose}  직전값 유지 {stale}  명령 동결 {frozen}"
          f"  (전체 {n})")
    if (stale + no_pose) / n > 0.1:
        print("[경고] 자세 끊김이 10% 를 넘는다. 카메라를 추가하거나 각도를 바꿀 것."
              " 5b.10 의 검출 성공률 시험을 먼저 하십시오.")
    pct = over / n * 100
    print(f"예산 초과 스텝 {over}/{n} = {pct:.1f}%")
    if pct > 1:
        print("[경고] 지연이 K=2 이상으로 커진다. 카메라 해상도를 낮추거나"
              " 검출 파라미터를 줄일 것. 가이드 9.4 절 참조.")
    if cam:
        cam.close()


# 큐브 국소 법선. cube_board.FACES 및 face_up.py 의 면 규약과 같은 순서다.
_NORMALS = np.array([[0., 0, 1], [0, 0, -1], [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0]])


def _face_error(cube_quat, face):
    """목표 면 법선이 월드 +z 에서 벗어난 각도 [rad]. 스핀 불변."""
    R = ob.quat_to_mat(cube_quat)
    return float(np.arccos(np.clip((R @ _NORMALS[face])[2], -1.0, 1.0)))


def _face_up(cube_quat):
    R = ob.quat_to_mat(cube_quat)
    return int(np.argmax([(R @ n)[2] for n in _NORMALS]))


def _mat2quat(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        return np.array([0.25 * s, (R[2, 1] - R[1, 2]) / s,
                         (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, 0.25 * s,
                         (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s])
    if i == 1:
        s = np.sqrt(1 - R[0, 0] + R[1, 1] - R[2, 2]) * 2
        return np.array([(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s,
                         0.25 * s, (R[1, 2] + R[2, 1]) / s])
    s = np.sqrt(1 - R[0, 0] - R[1, 1] + R[2, 2]) * 2
    return np.array([(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
                     (R[1, 2] + R[2, 1]) / s, 0.25 * s])


if __name__ == "__main__":
    main()
