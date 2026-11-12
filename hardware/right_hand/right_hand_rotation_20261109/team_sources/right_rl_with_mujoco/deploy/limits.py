"""시뮬 env 에서 자동 생성. 손으로 고치지 말 것.

hyperext_floor=-0.2, joint_limit_margin=0.0 기준의 안전 범위다.
pip/dip 하한이 -0.2 로 올라가 있다(과신전 차단). 실기 클리핑도 같아야 한다.
"""
import numpy as np

JOINT_NAMES = ['if_mcp', 'if_rot', 'if_pip', 'if_dip', 'mf_mcp', 'mf_rot', 'mf_pip', 'mf_dip', 'rf_mcp', 'rf_rot', 'rf_pip', 'rf_dip', 'th_cmc', 'th_axl', 'th_mcp', 'th_ipl']

SAFE_LOWERS = np.array([-0.314, -1.047, -0.2, -0.2, -0.314, -1.047, -0.2, -0.2, -0.314, -1.047, -0.2, -0.2, -0.349, -0.47, -1.2, -1.34])

SAFE_UPPERS = np.array([2.23, 1.047, 1.885, 2.042, 2.23, 1.047, 1.885, 2.042, 2.23, 1.047, 1.885, 2.042, 2.094, 2.443, 1.9, 1.88])

DEFAULT_POSE = np.array([0.8, 0.0, 0.8, 0.8, 0.8, 0.0, 0.8, 0.8, 0.8, 0.0, 0.8, 0.8, 0.8, 0.8, 0.8, 0.0])

ACTION_SCALE = 0.2
CTRL_DT = 0.05
SETTLE_STEPS = 0
SUCCESS_THRESHOLD = 0.15
POSITION_THRESHOLD = 0.015
HOLD_STEPS = 20
