"""Preserve the actually used quarter-turn prefix and append the fixed real grip.

Offline candidate construction and hand-geometry comparison, no device access.
"""
from pathlib import Path
import hashlib,json
import mujoco,numpy as np
from hand_motor_mapping import ticks_to_sim
from research_rotation_body_hold_20261104 import inspect_for,native

ROOT=Path(__file__).resolve().parent
FIRST=ROOT/'right_hand_rotation_20261104/forward_hold/candidate.npz'
REFERENCE=ROOT/'right_hand_rotation_20261117/user_measured_grip/reference.json'
MODEL=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
OUT=ROOT/'right_hand_rotation_20261118/quarter_turn_fixed_grip'


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def interpolate(start,target):
    n=max(1,int(np.ceil(max(abs(target-start))/2)))
    return np.rint(start+(target-start)*np.arange(1,n+1)[:,None]/n).astype(int)


def main():
    old=dict(np.load(FIRST,allow_pickle=False))
    grip=np.array(json.loads(REFERENCE.read_bytes())['holding_goal_ticks_readonly'],dtype=int)
    assert len(old['goals'])==1417
    end=old['goals'][-1].copy()
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':MODEL.read_bytes()})
    d=mujoco.MjData(m);_,_,check=inspect_for(m)
    lo,hi,_,_=native(m);hi[10]=max(hi[10],int(grip[10]))
    def geometry(q):
        d.qpos[:16]=ticks_to_sim(q);d.ctrl[:]=d.qpos[:16];d.qvel[:]=0
        mujoco.mj_fwdPosition(m,d)
        support,own=check(d)
        return support,own
    endpoint_support,endpoint_self=geometry(grip)
    final_baseline=dict(endpoint_self)
    old_support,old_self=geometry(end)
    baseline={k:max(final_baseline.get(k,0),old_self.get(k,0)) for k in set(final_baseline)|set(old_self)}
    variants={'direct':[('fixed_grip_transfer',grip.copy())]}
    receiver=end.copy();receiver[4:8]=np.rint((end[4:8]+grip[4:8])/2)
    receiver[12:16]=np.rint((end[12:16]+grip[12:16])/2)
    transfer=grip.copy();transfer[12:16]=receiver[12:16]
    variants['receiving']=[('receiving_index_ring',receiver),('thumb_middle_transfer',transfer),('fixed_grip_close',grip.copy())]
    receive=end.copy();receive[4:8]=np.rint((end[4:8]+grip[4:8])/2)
    middle=receive.copy();middle[8:12]=grip[8:12]
    thumb=middle.copy();thumb[:8]=grip[:8]
    variants['staged']=[('receiving_index',receive),('middle_receiving',middle),('thumb_index_transfer',thumb),('fixed_grip_close',grip.copy())]
    OUT.mkdir(parents=True,exist_ok=True);summaries=[]
    for name,stages in variants.items():
        previous=end.copy();pieces=[];labels=[]
        for label,target in stages:
            rows=interpolate(previous,target);pieces.extend(rows);labels.extend([label]*len(rows));previous=target
        pieces.extend([grip.copy()]*134);labels.extend(['fixed_grip_hold']*134)
        bridge=np.array(pieces,dtype=int);whole=np.vstack([old['goals'],bridge])
        phases=np.r_[old['phases'].astype('<U64'),np.array(labels,dtype='<U64')]
        all_goals=np.vstack([old['start'],whole])
        assert np.array_equal(whole[:1417],old['goals']) and np.array_equal(whole[-1],grip)
        assert max(abs(np.diff(all_goals,axis=0)).ravel())<=2
        assert np.all(all_goals>=lo) and np.all(all_goals<=hi)
        worst_self={};worst_support={};worst_delta=0.;worst_row=None
        for index,q in enumerate(bridge):
            support,own=geometry(q)
            for k,v in own.items():worst_self[k]=max(worst_self.get(k,0.),float(v))
            for k,v in support.items():worst_support[k]=max(worst_support.get(k,0.),float(v))
            delta=max((float(v)-baseline.get(k,0.) for k,v in own.items()),default=0.)
            if delta>worst_delta:worst_delta=delta;worst_row=index
        out=OUT/name;out.mkdir(exist_ok=True)
        np.savez_compressed(out/'candidate.npz',start=old['start'],goals=whole,phases=phases)
        report=dict(kind='observed_quarter_turn_then_fixed_physical_grip_candidate',variant=name,
            first_turn_goal_count=1417,first_turn_goals_exactly_preserved=True,
            fixed_grip_target_exactly_preserved=True,total_goals=len(whole),bridge_goals=len(bridge),
            maximum_goal_step_ticks=2,scoped_range_passed=True,
            worst_self_mm=worst_self,worst_support_mm=worst_support,
            fixed_grip_static_self_mm=final_baseline,largest_new_self_depth_mm=worst_delta,
            worst_new_self_bridge_index=worst_row,
            first_source_sha256=sha(FIRST),grip_reference_sha256=sha(REFERENCE),model_sha256=sha(MODEL),
            candidate_sha256=sha(out/'candidate.npz'),source_script_sha256=sha(Path(__file__)),
            hand_geometry_only=True,cube_transfer_verified=False,final_cube_angle_verified=False,
            physical_execution=False,hardware_execution_ready=False,
            requested_final_cube_angle_degrees=90,final_cube_angle_tolerance_degrees=5)
        (out/'construction_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        summary={k:v for k,v in report.items() if k in ('variant','total_goals','bridge_goals','worst_self_mm','worst_support_mm','largest_new_self_depth_mm','worst_new_self_bridge_index')}
        summaries.append(summary)
    (OUT/'construction_summary.json').write_text(json.dumps(summaries,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(summaries))


if __name__=='__main__':main()
