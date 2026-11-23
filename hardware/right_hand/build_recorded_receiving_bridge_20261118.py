"""Keep the full recorded turn and fixed final grip; use the real loose-pose geometry."""
from pathlib import Path
import hashlib,json
import mujoco,numpy as np
from build_quarter_fixed_grip_20261118 import ROOT,FIRST,REFERENCE,MODEL,OUT,interpolate
from hand_motor_mapping import ticks_to_sim
from research_rotation_body_hold_20261104 import native,inspect_for


def main():
    first=np.load(FIRST,allow_pickle=False)
    record_path=OUT/'receiving_reference.json';record=json.loads(record_path.read_bytes())
    actual=np.array(record['actual_ticks'],dtype=int)
    grip=np.array(json.loads(REFERENCE.read_bytes())['holding_goal_ticks_readonly'],dtype=int)
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':MODEL.read_bytes()})
    lo,hi,_,_=native(m);hi[10]=max(hi[10],grip[10])
    receiving=actual.copy()
    adjustments=[]
    for i in np.flatnonzero(receiving<np.ceil(lo)):
        if i not in (7,11,15):raise RuntimeError('Unexpected passive range exception')
        receiving[i]=int(np.ceil(lo[i]))+4
        adjustments.append(dict(id=int(i+1),measured=int(actual[i]),proposed=int(receiving[i])))
    if np.any(receiving>hi):raise RuntimeError('Measured receiving geometry above target limits')
    end=first['goals'][-1].copy()
    fingers=end.copy();fingers[4:]=receiving[4:]
    lower=end.copy();lower[[7,11,15]]=receiving[[7,11,15]]
    variants={'recorded_receiving':[('receiving_transfer',receiving)],
        'recorded_fingers_first':[('receiving_fingers',fingers),('receiving_thumb',receiving)],
        'recorded_lower_first':[('receiving_bases',lower),('receiving_fold',receiving)]}
    _,_,check=inspect_for(m);d=mujoco.MjData(m)
    def geometry(q):
        d.qpos[:16]=ticks_to_sim(q);d.ctrl[:]=d.qpos[:16];mujoco.mj_fwdPosition(m,d)
        support,own=check(d);return dict(support),dict(own)
    support_baseline,own_baseline=geometry(grip)
    receiving_support,receiving_own=geometry(receiving)
    for k,v in receiving_support.items():support_baseline[k]=max(support_baseline.get(k,0.),float(v))
    for k,v in receiving_own.items():own_baseline[k]=max(own_baseline.get(k,0.),float(v))
    reports=[]
    for name,stages in variants.items():
        rows=[];labels=[];previous=end.copy()
        for phase,target in stages+[('fixed_grip_close',grip)]:
            part=interpolate(previous,target);rows.extend(part);labels.extend([phase]*len(part));previous=target
        rows.extend([grip.copy()]*134);labels.extend(['fixed_grip_hold']*134)
        bridge=np.array(rows,dtype=int);whole=np.vstack([first['goals'],bridge])
        assert np.array_equal(whole[:1417],first['goals']) and np.array_equal(whole[-1],grip)
        assert np.all(whole>=lo) and np.all(whole<=hi)
        assert np.max(abs(np.diff(np.vstack([first['start'],whole]),axis=0)))<=2
        own_worst={};support_worst={}
        for q in bridge:
            support,own=geometry(q)
            for k,v in support.items():support_worst[k]=max(support_worst.get(k,0.),float(v))
            for k,v in own.items():own_worst[k]=max(own_worst.get(k,0.),float(v))
        folder=OUT/name;folder.mkdir(exist_ok=True)
        np.savez_compressed(folder/'candidate.npz',start=first['start'],goals=whole,
                            phases=np.r_[first['phases'].astype('<U64'),np.array(labels,dtype='<U64')])
        report=dict(variant=name,total_goals=len(whole),first1417_exactly_preserved=True,fixed_grip_exactly_preserved=True,
            receiving_geometry_source=str(record_path.relative_to(ROOT)),receiving_geometry_sha256=hashlib.sha256(record_path.read_bytes()).hexdigest(),
            raw_OFF_values_not_claimed_as_loaded_goals=True,receiving_proposal_ticks=receiving.tolist(),
            proposed_inward_adjustments=adjustments,new_target_limits_added=False,
            hand_self_worst_mm=own_worst,support_worst_mm=support_worst,
            new_self_depth_over_recorded_endpoints_mm=max((float(v)-own_baseline.get(k,0.) for k,v in own_worst.items()),default=0.),
            new_support_depth_over_recorded_endpoints_mm=max((float(v)-support_baseline.get(k,0.) for k,v in support_worst.items()),default=0.),
            candidate_sha256=hashlib.sha256((folder/'candidate.npz').read_bytes()).hexdigest(),hardware_execution_ready=False)
        (folder/'construction_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');reports.append(report)
    print(json.dumps(reports))


if __name__=='__main__':main()
