"""Static-only geometry comparison for the local ID12 timing change."""
from pathlib import Path
import json
import mujoco,numpy as np
from hand_motor_mapping import ticks_to_sim
from research_rotation_body_hold_20261104 import inspect_for

ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261118/grip_rotation_return'

def main():
    path=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':path.read_bytes()});d=mujoco.MjData(m);check=inspect_for(m)[2]
    old=np.load(ROOT/'right_hand_rotation_20261118/quarter_turn_fixed_grip/tip_first_lower_seat/candidate.npz',allow_pickle=False)['goals'][1417:]
    new=np.load(BASE/'candidate.npz',allow_pickle=False)['goals'][2505:]
    def worst(rows):
        result=[{},{}]
        for q in rows:
            d.qpos[:16]=ticks_to_sim(q);mujoco.mj_fwdPosition(m,d)
            for dst,values in zip(result,check(d)):
                for pair,value in values.items():dst[pair]=max(dst.get(pair,0.),float(value))
        return result
    before=worst(old);after=worst(new)
    extra=[{p:depth-before[j].get(p,0.) for p,depth in values.items() if depth>before[j].get(p,0.)+1e-9} for j,values in enumerate(after)]
    result=dict(kind='ID12_32tick_early_lift_static_comparison',before_support_mm=before[0],before_self_mm=before[1],
        after_support_mm=after[0],after_self_mm=after[1],added_support_mm=extra[0],added_self_mm=extra[1],
        colliders_unchanged=True,cube_motion_evaluated=False,physical_execution=False)
    (BASE/'middle_root_lead_geometry.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result))

if __name__=='__main__':main()
