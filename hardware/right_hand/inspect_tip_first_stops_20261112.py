"""Summarize the actual joint responsible for each offline stop."""
import json
from pathlib import Path
import numpy as np
from hand_motor_mapping import sim_to_ticks
ROOT=Path(__file__).resolve().parent
base=ROOT/'right_hand_rotation_20261112/tip_first_transfer_v1'
for folder in sorted(base.glob('fraction*')):
    z=np.load(folder/'end_state.npz',allow_pickle=False);goal=sim_to_ticks(z['ctrl']);actual=sim_to_ticks(z['qpos'][:16]);error=abs(goal-actual)
    print(json.dumps(dict(fraction=folder.name,worst=[dict(id=int(j+1),goal=int(goal[j]),actual=int(actual[j]),error_ticks=int(error[j])) for j in np.argsort(error)[-3:][::-1]])))
