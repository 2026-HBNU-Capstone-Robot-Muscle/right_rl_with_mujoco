"""Static diagnostic of the direct return; no device or collider changes."""
import hashlib,json,numpy as np,mujoco
from build_vertical_direct_grip_20261118 import ROOT,BASE,grip_entry
from hand_motor_mapping import ticks_to_sim
from research_rotation_body_hold_20261104 import inspect_for

def main():
    model=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':model.read_bytes()});d=mujoco.MjData(m);check=inspect_for(m)[2]
    new=np.load(BASE/'candidate.npz',allow_pickle=False)
    old=np.load(ROOT/'right_hand_rotation_20261118/vertical_rotation_return/candidate.npz',allow_pickle=False)
    review=json.loads((BASE/'construction_review.json').read_bytes())
    def worst(rows):
        result=[{},{}]
        for q in rows:
            d.qpos[:16]=ticks_to_sim(q);mujoco.mj_fwdPosition(m,d)
            for dst,values in zip(result,check(d)):
                for pair,value in values.items():dst[pair]=max(dst.get(pair,0.),float(value))
        return result
    baseline=worst(np.vstack([old['start'],old['goals']]))
    actual=worst(new['goals'][review['forward_goals']:])
    added=[{p:v-baseline[i].get(p,0.) for p,v in values.items() if v>baseline[i].get(p,0.)+1e-9}
           for i,values in enumerate(actual)]
    clear=json.loads((BASE/'thumb_clear_reference.json').read_bytes())
    snapshot=json.loads((ROOT/clear['source']).read_bytes())['snapshots'][-1]
    entry,labels=grip_entry(snapshot['goal'],new['goals'][-1])
    observed=worst(np.array([snapshot['goal'],snapshot['position']]))
    entry_depths=worst(entry)
    entry_added=[{p:v-max(baseline[i].get(p,0.),observed[i].get(p,0.)) for p,v in values.items()
        if v>max(baseline[i].get(p,0.),observed[i].get(p,0.))+1e-9} for i,values in enumerate(entry_depths)]
    result=dict(kind='direct_return_static_comparison',candidate_sha256=hashlib.sha256((BASE/'candidate.npz').read_bytes()).hexdigest(),
        baseline_support_mm=baseline[0],baseline_self_mm=baseline[1],
        direct_return_support_mm=actual[0],direct_return_self_mm=actual[1],
        added_support_mm=added[0],added_self_mm=added[1],colliders_unchanged=True,
        entry_observed_support_mm=observed[0],entry_observed_self_mm=observed[1],
        entry_added_support_mm=entry_added[0],entry_added_self_mm=entry_added[1],
        cube_motion_evaluated=False,physical_execution=False)
    (BASE/'static_review.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False))

if __name__=='__main__':main()
