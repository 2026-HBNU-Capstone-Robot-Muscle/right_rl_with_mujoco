"""Static geometry only; compare new handover to recorded vertical poses."""
import json
import mujoco,numpy as np
from vertical_transfer_plan_20261118 import ROOT,BASE,PITCH,reference
from hand_motor_mapping import ticks_to_sim
from research_rotation_body_hold_20261104 import inspect_for
import leap_measured_grip_hold_20261117 as pose

def main():
    path=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':path.read_bytes()});d=mujoco.MjData(m);check=inspect_for(m)[2]
    old=np.load(PITCH,allow_pickle=False);new=np.load(BASE/'candidate.npz',allow_pickle=False)
    def worst(rows):
        result=[{},{}]
        for q in rows:
            d.qpos[:16]=ticks_to_sim(q);mujoco.mj_fwdPosition(m,d)
            for dst,values in zip(result,check(d)):
                for pair,value in values.items():dst[pair]=max(dst.get(pair,0.),float(value))
        return result
    baseline=worst(np.vstack([pose.reference()[2],reference(),old['start'],old['goals']]))
    actual=worst(new['goals']);extra=[{p:v-baseline[j].get(p,0.) for p,v in a.items() if v>baseline[j].get(p,0.)+1e-9} for j,a in enumerate(actual)]
    result=dict(kind='vertical_handover_static_diagnostic',recorded_pose_baseline_support_mm=baseline[0],
        recorded_pose_baseline_self_mm=baseline[1],new_path_support_mm=actual[0],new_path_self_mm=actual[1],
        added_support_mm=extra[0],added_self_mm=extra[1],colliders_unchanged=True,
        cube_dynamics_evaluated=False,physical_execution=False)
    (BASE/'static_review.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result))

if __name__=='__main__':main()
