"""드라이버 경계면. 시뮬 규약 <-> 이 손의 하드웨어 규약.

이 파일이 순열과 오프셋을 아는 **유일한 곳**이다. 위쪽(정책, 관측 조립, 이력,
클리핑, 목표 관리)은 전부 시뮬 규약으로 돌고, 아래쪽만 모터 ID 1~16 과 +pi 다.

왜 여기인가 (학습 코드에 넣으면 안 되는 이유)
  1. 신경망 1층이 입력 순서에, 마지막 층이 출력 순서에 묶여 있다. 시뮬의
     JOINT_NAMES 순서를 바꾸면 학습한 체크포인트가 전부 무효가 된다.
  2. 순서를 참조하는 곳이 env 안에 흩어져 있다 - hyperext_floor 인덱스
     [2,3,6,7,10,11], _safe_lowers/uppers, 도메인 무작위화의 hand_qids,
     보상의 hand_pose. 하나만 놓쳐도 오류 없이 조용히 틀린다.
  3. 순열이 한 곳에 있으면 개루프 재생으로 그 한 곳만 검증하면 끝난다.

매핑 근거
  leap_python/main.py 의 배선 주석: 이 손은 ID 1~16 을 공식 순서의 역순으로
  부여했다(엄지->약지->중지->검지, 각 손가락 DIP->PIP->MCPFwd->MCPSide).
  leap_hand_utils.py 의 LEAPsim_limits 16개 범위를 우리 MJCF 범위와 대조해
  손가락 안에서 일대일로 확정했다. 우리 시뮬은 손바닥에서 바깥 순서(굴곡 먼저)
  이고 공식은 Side 를 먼저 세므로, 손가락 3개는 앞 두 개가 교차한다.

부호
  leap_hand_utils.py: "모든 회전 방향 부호는 두 손 사이에 같다. 오프셋만 다르다."
  범위가 비대칭인 13개는 그 일치로 확인됐다. **if_rot, mf_rot, rf_rot 만
  +-1.047 로 대칭이라 미확인이다.** deploy/check_joints.py 로 실기에서 볼 것.
"""
import numpy as np

# 시뮬 관절 i 가 read_pos()/write_desired_pos() 배열의 몇 번째인가
SIM2ARR = np.array([14, 15, 13, 12, 10, 11, 9, 8, 6, 7, 5, 4, 3, 2, 1, 0])
OFFSET = 3.14159            # LEAPsim -> LEAPhand
MOTORS = list(range(1, 17))  # 이 손의 모터 ID

SIM_JOINT_NAMES = [
    "if_mcp", "if_rot", "if_pip", "if_dip",
    "mf_mcp", "mf_rot", "mf_pip", "mf_dip",
    "rf_mcp", "rf_rot", "rf_pip", "rf_dip",
    "th_cmc", "th_axl", "th_mcp", "th_ipl",
]
# 부호가 아직 확인되지 않은 관절 (대칭 범위라 범위 대조로 못 가린다)
UNVERIFIED_SIGN = [1, 5, 9]      # if_rot, mf_rot, rf_rot

# 실기 게인. 옆흔들림 모터만 0.75 배인데 시뮬은 16개가 같은 kp=3.0 이다.
KP, KD, CURR_LIM = 600, 200, 450
SIDE_MOTOR_IDS = [8, 12, 16]     # 검지/중지/약지 MCP Side (엄지 side=4 는 제외)


def sim_to_hw(q_sim):
    """시뮬 규약 16개 -> 서보에 쓸 배열 16개."""
    hw = np.empty(16, dtype=float)
    hw[SIM2ARR] = np.asarray(q_sim, dtype=float) + OFFSET
    return hw


def hw_to_sim(hw):
    """서보에서 읽은 배열 16개 -> 시뮬 규약 16개."""
    return np.asarray(hw, dtype=float)[SIM2ARR] - OFFSET


class LeapHW:
    """dynamixel_sdk 위의 얇은 층. 규약 변환은 전부 여기서 끝난다.

    leap_python 의 DynamixelClient(전송 계층)만 재사용한다. set_ones/set_allegro
    같은 **규약 변환 함수는 쓰지 않는다** - 그것들은 다른 좌표계를 가정한다.
    """

    def __init__(self, port="/dev/ttyUSB0", baud=3000000,
                 kp=KP, kd=KD, curr_lim=CURR_LIM, latency_check=True):
        from leap_hand_utils.dynamixel_client import DynamixelClient
        self.c = DynamixelClient(MOTORS, port, baud)
        self.c.connect()
        n = len(MOTORS)
        self.c.sync_write(MOTORS, np.ones(n) * 5, 11, 1)      # position-current 모드
        # **토크를 켜기 전에 목표를 현재 위치로 맞춘다.**
        # 서보의 목표 위치는 전원을 꺼도 남는다. 지난 세션의 목표가 남은 채로
        # 토크를 켜면 손이 그 자세로 최대 속도로 튄다 - 손가락끼리 부딪히거나
        # 큐브를 튕겨낸다. 현재 위치를 목표로 써 두면 켜는 순간 정지 상태다.
        here = self.c.read_pos()
        self.c.write_desired_pos(MOTORS, here)
        self.c.set_torque_enabled(MOTORS, True)
        self.c.sync_write(MOTORS, np.ones(n) * kp, 84, 2)
        self.c.sync_write(MOTORS, np.ones(n) * kd, 80, 2)
        self.c.sync_write(SIDE_MOTOR_IDS, np.ones(3) * kp * 0.75, 84, 2)
        self.c.sync_write(SIDE_MOTOR_IDS, np.ones(3) * kd * 0.75, 80, 2)
        self.c.sync_write(MOTORS, np.ones(n) * curr_lim, 102, 2)
        self.curr_lim = curr_lim
        if latency_check:
            self.warn_latency(port)

    @staticmethod
    def warn_latency(port):
        """FTDI latency timer 가 16ms 기본이면 제어 주기(50ms)의 32% 를 먹는다.

        실측: 총 루프 지연이 K=3(0~150ms) 구간에 들어가면 연속 달성이 26% 떨어진다.
        """
        import os
        dev = os.path.basename(port)
        p = f"/sys/bus/usb-serial/devices/{dev}/latency_timer"
        try:
            v = int(open(p).read().strip())
        except Exception:
            print(f"[경고] latency_timer 를 못 읽었다: {p}")
            return
        if v > 2:
            print(f"[경고] {dev} latency_timer = {v} ms. 1 로 낮출 것:")
            print(f"    echo 1 | sudo tee {p}")

    def torque_nm(self):
        """전류 상한에 대응하는 토크. MJCF 주석: 0.2196 = 600/1000*0.366."""
        return self.curr_lim / 1000.0 * 0.366

    def write_sim(self, q_sim):
        self.c.write_desired_pos(MOTORS, sim_to_hw(q_sim))

    def move_to(self, q_target, seconds=1.5, hz=50.0):
        """현재 자세에서 목표까지 선형으로 램프한다.

        정지 상태에서 계단 명령을 주면 서보가 최대 속도로 달린다. 시작 자세로
        갈 때는 반드시 이걸 쓸 것. 제어 루프 안에서는 정책이 낸 증분이 이미
        작으므로(action_scale 0.2 rad) 램프가 필요 없다.
        """
        import time
        q0 = self.read_sim()
        q1 = np.clip(np.asarray(q_target, float), -np.pi, np.pi)
        n = max(int(seconds * hz), 1)
        for k in range(1, n + 1):
            self.write_sim(q0 + (q1 - q0) * (k / n))
            time.sleep(1.0 / hz)

    def read_sim(self):
        return hw_to_sim(self.c.read_pos())

    def read_sim_vel_cur(self):
        p, v, cur = self.c.read_pos_vel_cur()
        return hw_to_sim(p), np.asarray(v)[SIM2ARR], np.asarray(cur)[SIM2ARR]

    def torque_off(self):
        self.c.set_torque_enabled(MOTORS, False)
