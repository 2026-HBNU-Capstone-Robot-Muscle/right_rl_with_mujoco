"""Static comparison of the new receiving bridge against its recorded endpoints."""
import json
import mujoco,numpy as np
from build_vertical_return_20261118 import ROOT,BASE,TRANSFER,GRIP_RETURN,TAUGHT_PALM
from hand_motor_mapping import ticks_to_sim
from research_rotation_body_hold_20261104 import inspect_for

def main():
    model=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':model.read_bytes()});d=mujoco.MjData(m);check=inspect_for(m)[2]
    new=np.load(BASE/'candidate.npz',allow_pickle=False);old=np.load(TRANSFER,allow_pickle=False);h=np.load(GRIP_RETURN,allow_pickle=False)
    tail=int(np.flatnonzero(h['phases']=='return_transfer_middle_clear')[0])
    def worst(rows):
        result=[{},{}]
        for q in rows:
            d.qpos[:16]=ticks_to_sim(q);mujoco.mj_fwdPosition(m,d)
            for dst,values in zip(result,check(d)):
                for pair,value in values.items():dst[pair]=max(dst.get(pair,0.),float(value))
        return result
    baseline=worst(np.vstack([new['start'],old['start'],old['goals'],h['goals'][tail:]]))
    taught=json.loads(TAUGHT_PALM.read_bytes())
    measured=worst(np.array([taught['goal_ticks'],taught['actual_ticks']]))
    actual=worst(new['goals']);extra=[{p:v-baseline[j].get(p,0.) for p,v in a.items() if v>baseline[j].get(p,0.)+1e-9} for j,a in enumerate(actual)]
    beyond=[{p:v-max(baseline[j].get(p,0.),measured[j].get(p,0.)) for p,v in a.items()
        if v>max(baseline[j].get(p,0.),measured[j].get(p,0.))+1e-9} for j,a in enumerate(actual)]
    result=dict(kind='vertical_return_receiving_bridge_static_diagnostic',baseline_support_mm=baseline[0],baseline_self_mm=baseline[1],
        new_support_mm=actual[0],new_self_mm=actual[1],added_support_mm=extra[0],added_self_mm=extra[1],
        taught_pose_support_mm=measured[0],taught_pose_self_mm=measured[1],
        beyond_recorded_and_taught_support_mm=beyond[0],beyond_recorded_and_taught_self_mm=beyond[1],
        measured_pose_does_not_validate_support_model=True,
        baseline_includes_recorded_middle_clear_thumb_push_and_grip=True,
        colliders_unchanged=True,cube_motion_evaluated=False,physical_execution=False)
    (BASE/'static_review.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result))

if __name__=='__main__':main()
