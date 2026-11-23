"""Static hand-only check for the EMPTY preparation, using recorded real start angles."""
from pathlib import Path
import json
import mujoco,numpy as np
import leap_measured_grip_hold_20261117 as pose
from hand_motor_mapping import ticks_to_sim
from research_rotation_body_hold_20261104 import inspect_for

ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261118/quarter_turn_fixed_grip'


def main():
    recorded=json.loads((BASE/'receiving_reference.json').read_bytes())
    start=np.array(recorded['actual_ticks'],dtype=int)
    placement=np.load(ROOT/'right_hand_rotation_20261104/forward_hold/candidate.npz')['start']
    rows=pose.path(start,placement)
    p=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':p.read_bytes()})
    d=mujoco.MjData(m);_,_,check=inspect_for(m)
    def measure(q):
        d.qpos[:16]=ticks_to_sim(q);d.ctrl[:]=d.qpos[:16];mujoco.mj_fwdPosition(m,d)
        support,own=check(d);return dict(support),dict(own)
    baseline_support,baseline_own=measure(start);bsp,bown=measure(placement)
    for key,v in bsp.items():baseline_support[key]=max(baseline_support.get(key,0.),v)
    for key,v in bown.items():baseline_own[key]=max(baseline_own.get(key,0.),v)
    own_worst={};support_worst={}
    for row in rows:
        support,own=measure(row)
        for k,v in support.items():support_worst[k]=max(support_worst.get(k,0.),float(v))
        for k,v in own.items():own_worst[k]=max(own_worst.get(k,0.),float(v))
    result=dict(kind='empty_preparation_static_hand_geometry',goals=len(rows),
        known_start_actual_ticks=start.tolist(),placement_target=placement.tolist(),
        maximum_self_mm=own_worst,maximum_support_mm=support_worst,
        largest_new_self_depth_mm=max((v-baseline_own.get(k,0.) for k,v in own_worst.items()),default=0.),
        largest_new_support_depth_mm=max((v-baseline_support.get(k,0.) for k,v in support_worst.items()),default=0.),
        cube_must_be_absent=True,physical_execution=False,does_not_validate_arbitrary_other_start_poses=True)
    (BASE/'empty_start_review.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result))


if __name__=='__main__':main()
