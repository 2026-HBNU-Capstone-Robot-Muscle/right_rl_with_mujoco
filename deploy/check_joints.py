"""1단계 개루프 시험 - 큐브 없이. 매핑·부호·지연을 분리해서 잡는다.

정책이 개입하지 않으므로 여기서 나오는 오차는 전부 규약 문제다.
여기서 안 맞으면 뒤는 볼 필요가 없다.

세 가지를 본다
  1. 매핑   : 시뮬 관절 하나만 움직여 실제로 그 손가락 그 마디가 움직이는가
  2. 부호   : if_rot/mf_rot/rf_rot 이 옆으로 벌어지는가 굽는가
              (범위가 +-1.047 로 대칭이라 범위 대조로는 못 가린 3개다)
  3. 지연   : 계단 명령을 주고 엔코더가 반응할 때까지의 시간
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import limits
import leap_hw


def sweep(hw, i, amp=0.6, hold=1.2):
    q = limits.DEFAULT_POSE.copy()
    hw.write_sim(q); time.sleep(0.8)
    tgt = float(np.clip(q[i] + amp, limits.SAFE_LOWERS[i], limits.SAFE_UPPERS[i]))
    q[i] = tgt
    hw.write_sim(q); time.sleep(hold)
    meas = hw.read_sim()
    hw.write_sim(limits.DEFAULT_POSE); time.sleep(0.6)
    return tgt, meas[i], meas


def latency(hw, i, amp=0.5, timeout=0.5, thresh=0.02):
    """계단 명령 후 엔코더가 움직이기 시작할 때까지의 시간 [ms]."""
    q = limits.DEFAULT_POSE.copy()
    hw.write_sim(q); time.sleep(0.8)
    q0 = hw.read_sim()[i]
    q[i] = float(np.clip(q[i] + amp, limits.SAFE_LOWERS[i], limits.SAFE_UPPERS[i]))
    t0 = time.time()
    hw.write_sim(q)
    while time.time() - t0 < timeout:
        if abs(hw.read_sim()[i] - q0) > thresh:
            dt = (time.time() - t0) * 1000
            hw.write_sim(limits.DEFAULT_POSE); time.sleep(0.5)
            return dt
    hw.write_sim(limits.DEFAULT_POSE); time.sleep(0.5)
    return float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--mode", choices=["map", "sign", "latency"], default="sign")
    a = ap.parse_args()
    hw = leap_hw.LeapHW(port=a.port)
    print(f"토크 상한 {hw.torque_nm():.4f} Nm\n")

    if a.mode == "map":
        for i, n in enumerate(leap_hw.SIM_JOINT_NAMES):
            tgt, got, _ = sweep(hw, i)
            print(f"[{i:>2}] {n:<8} 명령 {tgt:+.3f} -> 측정 {got:+.3f}  "
                  f"오차 {got-tgt:+.3f}")
            input("     움직인 곳이 위 이름과 같습니까? Enter... ")

    elif a.mode == "sign":
        print("범위가 대칭이라 부호가 확인되지 않은 3개를 본다.")
        print("**옆으로 벌어지면 매핑이 맞고, 굽으면 틀린 것이다.**\n")
        for i in leap_hw.UNVERIFIED_SIGN:
            n = leap_hw.SIM_JOINT_NAMES[i]
            print(f"[{i}] {n}  (배열 위치 {leap_hw.SIM2ARR[i]}, "
                  f"모터 ID {leap_hw.SIM2ARR[i]+1})")
            for amp in (+0.6, -0.6):
                sweep(hw, i, amp=amp)
                print(f"    {amp:+.1f} rad 인가했습니다.")
                input("    벌어졌나요(옆) 굽었나요(앞)? 확인 후 Enter... ")

    else:
        print("계단 명령 -> 엔코더 반응까지의 시간. 총 루프 지연의 하한이다.")
        print("(카메라 지연은 여기 안 들어간다. 5b 절 참조)\n")
        for i in (0, 4, 8, 12):
            v = [latency(hw, i) for _ in range(5)]
            v = np.array(v)
            print(f"  {leap_hw.SIM_JOINT_NAMES[i]:<8} "
                  f"중앙값 {np.nanmedian(v):6.1f} ms   개별 "
                  + " ".join(f"{x:.0f}" for x in v))
        print(f"\n제어 주기 {limits.CTRL_DT*1000:.0f} ms 기준으로 "
              "1스텝 = K 1 증가. 9.4 절 표와 대조할 것.")
    hw.torque_off()


if __name__ == "__main__":
    main()
