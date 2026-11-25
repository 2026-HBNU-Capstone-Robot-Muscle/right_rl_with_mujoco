"""Conditional small closure search from recorded ON following error.

Diagnostic only. No fitted mounting, joint offsets, or motor gains. Never
exports an executable launcher; the provisional mount remains unverified.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib
import json
from pathlib import Path
import mujoco
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from sac_cube_model import set_cube_physics
from hand_motor_mapping import ticks_to_sim
from box_clearance import clearance

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'slow_grasp_demo_20261023_182105_573825'


def main():
    blob=(FOLDER/'on_paired_pose_review.json').read_bytes();paired=json.loads(blob)
    run=json.loads((FOLDER/'result.json').read_bytes())
    held=[s for s in run['samples'] if s['phase']=='stage_4_fixed_grasp_observation']
    base=np.array(held[-1]['goal']);actual=np.median([s['actual'] for s in held],axis=0)
    residual=actual-base
    poses=[h for h in paired['hypotheses'] if h['source'].endswith('grid_diagnostic') and h['palm_branch']==0]
    assert len(poses)>=3
    # Select median and two spatially separated measured observations without
    # ranking a pose by desired finger contact. Retain every original branch.
    centers=np.array([p['cube_center_world_m'] for p in poses]);median=np.median(centers,axis=0)
    indices=[int(np.argmin(np.linalg.norm(centers-median,axis=1))),int(np.argmin(centers[:,2])),int(np.argmax(centers[:,2]))]
    poses=[poses[i] for i in sorted(set(indices))]
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build_with_middle_thumb_hulls(env);set_cube_physics(m,side_m=.055,mass_kg=.1)
    d=mujoco.MjData(m);cube=m.geom('cube').id;tip_ids=[m.geom(t).id for t in ('if_tip','mf_tip','th_tip')]
    support={m.geom(n).id for n in ('support_glass','support_wood')};hand={m.body('palm').id}
    for i in range(m.nbody):
        if int(m.body_parentid[i]) in hand:hand.add(i)
    variables=np.array([0,1,2,4,6,8,10])
    bounds_record=json.loads((ROOT/'right_hand_rotation_20261023/native_motor_limits.json').read_bytes())
    lower=np.array(bounds_record['lower_ticks']);upper=np.array(bounds_record['upper_ticks'])

    def geometry(goal,offset,*,independent=False):
        state=goal+offset;d.qpos[:16]=ticks_to_sim(state);rows=[]
        for p in poses:
            R=np.array(p['cube_rotation_world']);center=np.array(p['cube_center_world_m'])
            d.qpos[-7:]=np.r_[center,Rotation.from_matrix(R).as_quat()[[3,0,1,2]]]
            d.qvel[:]=0;mujoco.mj_forward(m,d)
            gaps=[];euclidean=[]
            for g in tip_ids:
                gR=d.geom_xmat[g].reshape(3,3);center_tip=d.geom_xpos[g]
                axes=np.r_[R.T,gR.T,np.cross(R.T[:,None,:],gR.T[None,:,:]).reshape(9,3)]
                lengths=np.linalg.norm(axes,axis=1);axes=axes[lengths>1e-10]/lengths[lengths>1e-10,None]
                radii_cube=np.sum(abs(axes@R)*.0275,axis=1)
                radii_tip=np.sum(abs(axes@gR)*m.geom_size[g],axis=1)
                score=np.max(abs(axes@(center_tip-center))-radii_cube-radii_tip)*1000
                gaps.append(float(score))
                if independent:
                    gap,overlap=clearance(center,R,np.full(3,.0275),center_tip,gR,m.geom_size[g])
                    euclidean.append(dict(gap_mm=gap*1000,boxes_overlap=overlap))
            self_depth=support_depth=0.;non_tip=0.
            for c in d.contact:
                a,b=int(c.geom1),int(c.geom2);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
                if a!=cube and b!=cube:
                    if ha and hb:self_depth=max(self_depth,-c.dist*1000)
                    if (a in support and hb) or (b in support and ha):support_depth=max(support_depth,-c.dist*1000)
                elif a not in tip_ids and b not in tip_ids:non_tip=max(non_tip,-c.dist*1000)
            rows.append(dict(projected_separation_scores_mm=gaps,independent_euclidean=euclidean,self_overlap_mm=self_depth,
                support_overlap_mm=support_depth,other_cube_overlap_mm=non_tip))
        return rows

    baseline=geometry(base,residual);trials=[];best=None
    for budget in (16,32,64):
        lo=np.maximum(-budget,lower[variables]-base[variables]);hi=np.minimum(budget,upper[variables]-base[variables])
        def objective(x):
            goal=base.astype(float).copy();goal[variables]+=x
            rows=geometry(goal,residual);cost=0.
            for row,old in zip(rows,baseline):
                gap=np.array(row['projected_separation_scores_mm'])
                cost+=float(np.sum(np.maximum(gap-1.,0)**2+8*np.maximum(-gap-.75,0)**2))
                cost+=10000*(row['self_overlap_mm']**2+row['support_overlap_mm']**2)
                cost+=20*max(0.,row['other_cube_overlap_mm']-old['other_cube_overlap_mm']-.25)**2
            return cost+.0005*float(x@x)
        solution=minimize(objective,np.zeros(len(variables)),method='Powell',bounds=list(zip(lo,hi)),
            options=dict(maxiter=20,maxfev=1800,xtol=.4,ftol=1e-4))
        goal=base.copy();goal[variables]+=np.rint(solution.x).astype(int)
        rows=geometry(goal,residual,independent=True);raw_rows=geometry(goal,np.zeros(16),independent=True)
        contact_goal=all(max(d['gap_mm'] for d in r['independent_euclidean'])<=2
            and min(r['projected_separation_scores_mm'])>=-1 for r in rows)
        hand_clear=all(r['self_overlap_mm']<=1e-6 and r['support_overlap_mm']<=1e-6 for r in rows+raw_rows)
        record=dict(budget_ticks=budget,delta_ticks=(goal-base).tolist(),goal_ticks=goal.tolist(),
            predicted_actual_rows=rows,raw_command_rows=raw_rows,contact_goal_met=contact_goal,
            hand_and_support_endpoint_clear=hand_clear,objective=float(solution.fun),evaluations=solution.nfev)
        trials.append(record);print(json.dumps(record),flush=True)
        if contact_goal and hand_clear:best=record;break
    out=dict(status='conditional_small_closure_search_only',source_sha256=hashlib.sha256(blob).hexdigest(),
        source=FOLDER.name,base_goal_ticks=base.tolist(),measured_median_actual_ticks=actual.tolist(),
        measured_following_residual_ticks=residual.tolist(),moving_motor_ids=(variables+1).tolist(),
        initial_model_rows=baseline,pose_hypotheses_used=poses,trials=trials,selected=best,
        model_manifest=manifest,registration_verified=False,full_path_screened=False,
        joint_offsets_fitted=False,motor_current_or_gain_changed=False,live_sac=False,
        hardware_execution_allowed=False,hardware_commands_sent=0,
        limitations=['Primary palm branch and solved-grid scale are conditional inputs, not calibrated geometry.',
            'Following error is a recorded actuator response, not a fitted mechanical joint offset.',
            'SAT scores are projection separations, not Euclidean clearance or measured penetration depth.',
            'Final endpoint gaps use independent box feature distances; native distance zero is not treated as contact.',
            'Endpoint clearance does not establish a safe motion path or stable object manipulation.',
            'Operator grip observation and further offline validation are still needed.'])
    destination=FOLDER/'conditional_contact_closure_review.json'
    if destination.exists():
        old=destination.read_bytes();archive=FOLDER/('closure_native_distance_only_'+hashlib.sha256(old).hexdigest()[:16]+'.json')
        if not archive.exists():archive.write_bytes(old)
    destination.write_text(json.dumps(out,indent=2),encoding='utf-8')


if __name__=='__main__':main()
