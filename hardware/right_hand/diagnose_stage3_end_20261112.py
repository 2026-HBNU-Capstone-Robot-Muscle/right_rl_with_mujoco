"""One compact native endpoint diagnosis; no hardware imports."""
import json
import sys
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle'))
import mujoco
from native_cycle import assets, restore, Guard, SIM_TO_MOTOR_IDS, motor, save_json

folder = ROOT/'right_hand_rotation_20261112/stage3_preparation/relative_turn_contact_v2'
m, _, _, _, _ = assets('active_ccd50'); m.geom_friction[:, 0] = .2
z = dict(np.load(folder/'end_state.npz', allow_pickle=False)); d = restore(m, z); g = Guard(m)
start = {'command_ticks': motor(d.ctrl).tolist(), 'actual_range_headroom_deg': np.degrees(np.minimum(d.qpos[:16]-g.qlo,g.qhi-d.qpos[:16])).tolist(), 'sim_motor_ids': list(SIM_TO_MOTOR_IDS)}
for i in range(600):
    mujoco.mj_step(m, d); why = g.check(d)
    if why: break
excess = np.maximum(0.,np.maximum(g.qlo-d.qpos[:16], d.qpos[:16]-g.qhi))
report = {'source': str(folder.relative_to(ROOT)/'end_state.npz'), 'start': start, 'zero_hold_ms': i+1,
          'stop': why, 'range_excess_by_motor_deg': {str(k): float(v) for k,v in zip(SIM_TO_MOTOR_IDS,np.degrees(excess)) if v > 0}, 'terminal': g.last}
save_json(folder/'endpoint_diagnosis.json', report)
print(json.dumps({k:v for k,v in report.items() if k!='start'}))
