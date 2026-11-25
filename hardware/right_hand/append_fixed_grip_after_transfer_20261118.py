"""Extend existing full-turn/receiving transfers into the unchanged physical grip."""
from pathlib import Path
import hashlib,json
import mujoco,numpy as np
from build_quarter_fixed_grip_20261118 import ROOT,FIRST,REFERENCE,MODEL,OUT,interpolate
from hand_motor_mapping import ticks_to_sim
from research_rotation_body_hold_20261104 import inspect_for,native


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    original=np.load(FIRST,allow_pickle=False)
    grip=np.array(json.loads(REFERENCE.read_bytes())['holding_goal_ticks_readonly'],dtype=int)
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':MODEL.read_bytes()})
    d=mujoco.MjData(m);_,_,check=inspect_for(m);lo,hi,_,_=native(m);hi[10]=max(hi[10],grip[10])
    def geometry(q):
        d.qpos[:16]=ticks_to_sim(q);d.ctrl[:]=d.qpos[:16];d.qvel[:]=0;mujoco.mj_fwdPosition(m,d)
        support,own=check(d)
        return {k:float(v) for k,v in support.items()},{k:float(v) for k,v in own.items()}
    endpoint_support,endpoint_self=geometry(grip)
    sources={'tip_curl':ROOT/'right_hand_rotation_20261112/tip_first_trial_v2/candidate.npz',
             'support_curl':ROOT/'right_hand_rotation_20261112/support_first_trial/candidate.npz'}
    summary=[]
    for name,source in sources.items():
        z=np.load(source,allow_pickle=False);q=z['goals'];end=q[-1].copy()
        assert np.array_equal(q[:1417],original['goals'])
        fold=end.copy();fold[1]=grip[1];fold[0]=2450;fold[8:12]=grip[8:12]
        fold[4:8]=np.rint((end[4:8]+grip[4:8])/2);fold[12:16]=np.rint((end[12:16]+grip[12:16])/2)
        cage=fold.copy();cage[4:16]=grip[4:16]
        stages=[('fold_middle_thumb_to_palm',fold),('close_receiving_cage',cage),('fixed_thumb_wrap',grip)]
        bridge=[];labels=[];previous=end.copy()
        for phase,target in stages:
            rows=interpolate(previous,target);bridge.extend(rows);labels.extend([phase]*len(rows));previous=target
        bridge.extend([grip.copy()]*134);labels.extend(['fixed_grip_hold']*134)
        bridge=np.array(bridge);whole=np.vstack([q,bridge]);phases=np.r_[z['phases'].astype('<U64'),np.array(labels,dtype='<U64')]
        assert np.all(whole>=lo) and np.all(whole<=hi)
        assert np.max(abs(np.diff(np.vstack([z['start'],whole]),axis=0)))<=2
        worst_self={};worst_support={}
        for command in bridge:
            support,own=geometry(command)
            for k,v in support.items():worst_support[k]=max(worst_support.get(k,0.),v)
            for k,v in own.items():worst_self[k]=max(worst_self.get(k,0.),v)
        folder=OUT/name;folder.mkdir(parents=True,exist_ok=True)
        np.savez_compressed(folder/'candidate.npz',start=z['start'],goals=whole,phases=phases)
        report=dict(variant=name,first_turn_goal_count=1417,first_turn_goals_exactly_preserved=True,
            reused_transfer_source=source.relative_to(ROOT).as_posix(),reused_transfer_source_sha256=sha(source),
            fixed_grip_target_exactly_preserved=True,total_goals=len(whole),bridge_goals=len(bridge),
            maximum_goal_step_ticks=2,worst_self_mm=worst_self,worst_support_mm=worst_support,
            fixed_grip_static_self_mm=endpoint_self,fixed_grip_static_support_mm=endpoint_support,
            largest_new_self_depth_mm=max((v-endpoint_self.get(k,0.) for k,v in worst_self.items()),default=0.),
            largest_new_support_depth_mm=max((v-endpoint_support.get(k,0.) for k,v in worst_support.items()),default=0.),
            candidate_sha256=sha(folder/'candidate.npz'),grip_reference_sha256=sha(REFERENCE),
            source_script_sha256=sha(Path(__file__)),hardware_execution_ready=False)
        (folder/'construction_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        summary.append(report)
    print(json.dumps(summary))


if __name__=='__main__':main()
