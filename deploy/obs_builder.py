"""실기 입력에서 정책이 먹는 108차원 관측을 조립한다.

시뮬의 reorient._get_obs 를 그대로 옮긴 것이다. 순서·부호가 하나라도 다르면
정책은 오류 없이 엉뚱하게 움직인다. tests/test_obs_match.py 가 시뮬 env 의
출력과 부동소수점 수준에서 일치하는지 확인한다.

구성 (합 108)
    관절각                16
    관절 오차 이력 x3     48     (측정각 - 명령각)
    큐브 위치 오차 이력 x3  9     (손바닥 - 큐브)
    큐브 자세 오차 이력 x3 18     quat_to_mat(q_cube * inv(q_goal)).ravel()[3:]
    직전 행동             16
    자세 오차 스칼라        1     2*asin(|quat_diff.xyz|)

주의
  * 이력은 **0번이 최신**이다. roll 후 앞을 덮어쓴다.
  * 관절 오차는 측정각에서 **명령각**을 뺀다. 명령각은 직전에 서보로 보낸 값이다.
  * 큐브 위치 오차는 `손바닥 - 큐브` 다. 카메라가 주는 값이 `큐브 - 손마커` 이면
    **부호를 뒤집어야 한다.**
  * face_onehot_obs 는 꺼져 있다(확인함). 원핫은 붙지 않는다.
"""
import numpy as np

HISTORY_LEN = 3
NQ = 16
_S2 = np.sqrt(0.5)

# 목표 면 -> 목표 자세 쿼터니언 (face_up.py 의 _FACE_GOAL_QUATS 와 동일)
FACE_GOAL_QUATS = np.array([
    [1.0, 0.0, 0.0, 0.0],     # 0 +z 가 위
    [0.0, 1.0, 0.0, 0.0],     # 1 -z
    [_S2, 0.0, -_S2, 0.0],    # 2 +x
    [_S2, 0.0, _S2, 0.0],     # 3 -x
    [_S2, _S2, 0.0, 0.0],     # 4 +y
    [_S2, -_S2, 0.0, 0.0],    # 5 -y
])


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_inv(q):
    return np.array([q[0], -q[1], -q[2], -q[3]]) / float(q @ q)


def quat_to_mat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


class ObsBuilder:
    """이력 버퍼를 들고 매 스텝 108차원을 낸다."""

    def __init__(self, history_len=HISTORY_LEN):
        self.h = history_len
        self.reset()

    def reset(self):
        self.qpos_err_hist = np.zeros(self.h * NQ)
        self.cube_pos_err_hist = np.zeros(self.h * 3)
        self.cube_ori_err_hist = np.zeros(self.h * 6)
        self.last_act = np.zeros(NQ)

    def fill_ori_history(self, cube_quat, goal_quat):
        """목표가 바뀐 직후 옛 목표의 잔상을 지운다(연쇄 모드에서 필요)."""
        x = self._xmat_diff(cube_quat, goal_quat)
        self.cube_ori_err_hist = np.tile(x, self.h)

    @staticmethod
    def _xmat_diff(cube_quat, goal_quat):
        return quat_to_mat(quat_mul(cube_quat, quat_inv(goal_quat))).ravel()[3:]

    def build(self, q_meas, q_cmd, palm_pos, cube_pos, cube_quat, goal_quat):
        """q_meas/q_cmd: 시뮬 규약 16개(rad). palm_pos/cube_pos: m. quat: wxyz."""
        q_meas = np.asarray(q_meas, float)
        qe = q_meas - np.asarray(q_cmd, float)
        self.qpos_err_hist = np.roll(self.qpos_err_hist, NQ)
        self.qpos_err_hist[:NQ] = qe

        pe = np.asarray(palm_pos, float) - np.asarray(cube_pos, float)
        self.cube_pos_err_hist = np.roll(self.cube_pos_err_hist, 3)
        self.cube_pos_err_hist[:3] = pe

        qd = quat_mul(np.asarray(cube_quat, float), quat_inv(np.asarray(goal_quat, float)))
        self.cube_ori_err_hist = np.roll(self.cube_ori_err_hist, 6)
        self.cube_ori_err_hist[:6] = quat_to_mat(qd).ravel()[3:]

        qn = qd / np.linalg.norm(qd)
        ori_err = 2.0 * np.arcsin(min(np.linalg.norm(qn[1:]), 1.0))

        return np.concatenate([
            q_meas,                    # 16
            self.qpos_err_hist,        # 48
            self.cube_pos_err_hist,    # 9
            self.cube_ori_err_hist,    # 18
            self.last_act,             # 16
            [ori_err],                 # 1
        ])

    def set_last_act(self, a):
        self.last_act = np.asarray(a, float).copy()
