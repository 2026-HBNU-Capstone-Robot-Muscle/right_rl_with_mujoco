"""Small coordinated tip displacement from this trial's ON reference.

This prepares an offline candidate, not hardware approval. It preserves the
measured following error as an actuator-response diagnostic, not joint offsets.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib
import json
from pathlib import Path
import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from hand_motor_mapping import ticks_to_sim
from grasp_place_plan import endpoints

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'slow_grasp_demo_20261023_182105_573825'
OUT=ROOT/'right_hand_rotation_20261023'
SAMPLES=90
MAX_DELTA=64


def main():
    paired_blob=(SOURCE/'on_paired_pose_review.json').read_bytes();paired=json.loads(paired_blob)
    trial_blob=(SOURCE/'result.json').read_bytes();trial=json.loads(trial_blob)
    assert trial['status']=='slow_grasp_demo_complete' and trial['all_torque_off_verified'] and trial['settings_restored']
    hold=[s for s in trial['samples'] if s['phase']=='stage_4_fixed_grasp_observation']
    base=endpoints()[1];actual=np.median([s['actual'] for s in hold],axis=0);residual=actual-base
    poses=[p for p in paired['hypotheses'] if p['source'].endswith('grid_diagnostic') and p['palm_branch']==0]
    assert len(poses)==11
    centers=np.array([p['cube_center_world_m'] for p in poses]);center=np.median(centers,axis=0)
    axis=np.median([np.array(p['cube_rotation_world'])[:,2] for p in poses],axis=0);axis/=np.linalg.norm(axis)
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build_with_middle_thumb_hulls(env);d=mujoco.MjData(m)
    cube=m.geom('cube').id;m.geom_contype[cube]=m.geom_conaffinity[cube]=0
    support={m.geom(n).id for n in ('support_glass','support_wood')};hand={m.body('palm').id}
    for i in range(m.nbody):
        if int(m.body_parentid[i]) in hand:hand.add(i)
    tip_names=('if_tip','mf_tip','th_tip');tip_ids=[m.geom(n).id for n in tip_names]
    # Keep all finger roots and the entire ring finger fixed. Only reviewed
    # finger flexion/rotation joints may contribute to the small displacement.
    moving=np.array([0,1,2,4,5,6,8,9,10])
    limits=json.loads((OUT/'native_motor_limits.json').read_bytes())
    lower=np.array(limits['lower_ticks']);upper=np.array(limits['upper_ticks'])
    lo=np.maximum(-MAX_DELTA,lower[moving]-base[moving]);hi=np.minimum(MAX_DELTA,upper[moving]-base[moving])
    def tips(goal,offset):
        d.qpos[:16]=ticks_to_sim(goal+offset);d.qpos[-7:]=[0,0,2,1,0,0,0]
        d.qvel[:]=0;mujoco.mj_forward(m,d)
        return d.geom_xpos[tip_ids].copy()
    initial=tips(base,residual);attempts=[];chosen=None
    def path_overlaps(goals):
        self_max=support_max=0.
        for offset in (np.zeros(16),residual):
            for goal in goals:
                tips(goal,offset)
                for c in d.contact:
                    a,b=int(c.geom1),int(c.geom2)
                    ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
                    if ha and hb:self_max=max(self_max,-float(c.dist)*1000)
                    if (a in support and hb) or (b in support and ha):support_max=max(support_max,-float(c.dist)*1000)
        return self_max,support_max
    # Requested axis angle is a model target, never a physical success claim.
    for angle in (-5.,-4.,-3.):
        goals=[];previous=np.zeros(len(moving));errors=[]
        for step in range(1,SAMPLES+1):
            fraction=step/SAMPLES;smooth=fraction*fraction*(3-2*fraction)
            R=Rotation.from_rotvec(axis*np.radians(angle)*smooth).as_matrix()
            desired=(initial-center)@R.T+center
            def objective(x):
                goal=base.astype(float).copy();goal[moving]+=x
                position=(tips(goal,residual)-desired).ravel()*1000
                return np.r_[position,.015*x]
            solution=least_squares(objective,previous,bounds=(lo,hi),diff_step=.025,
                ftol=1e-8,xtol=1e-7,gtol=1e-7,max_nfev=60)
            previous=solution.x;goal=base.copy();goal[moving]+=np.rint(previous).astype(int)
            goals.append(goal);errors.append(np.linalg.norm(tips(goal,residual)-desired,axis=1)*1000)
        goals=np.array(goals);errors=np.array(errors)
        self_overlap,support_overlap=path_overlaps(goals)
        record=dict(target_yaw_degrees=angle,maximum_tip_target_error_mm=float(np.max(errors)),
            final_tip_target_error_mm=errors[-1].tolist(),maximum_goal_delta_ticks=int(np.max(abs(goals-base))),
            maximum_step_ticks=int(np.max(abs(np.diff(np.vstack([base,goals]),axis=0)))),
            raw_and_observed_response_static_self_overlap_mm=self_overlap,
            raw_and_observed_response_static_support_overlap_mm=support_overlap)
        attempts.append(record);print(json.dumps(record),flush=True)
        if (record['maximum_tip_target_error_mm']<=1.5 and record['maximum_step_ticks']<=2
                and self_overlap<=1e-6 and support_overlap<=1e-6):
            chosen=(record,goals);break
    if chosen is None:raise RuntimeError('No bounded small coordinated candidate found; no launcher generated')
    record,goals=chosen
    result=dict(version='same_grasp_small_yaw_candidate_v1',base_goal_ticks=base.tolist(),goal_ticks=goals.tolist(),
        source_trial=SOURCE.name,source_trial_sha256=hashlib.sha256(trial_blob).hexdigest(),
        source_ON_pairs_sha256=hashlib.sha256(paired_blob).hexdigest(),
        sampled_actual_ticks=actual.tolist(),recorded_following_residual_ticks=residual.tolist(),
        cube_center_world_m_candidate=center.tolist(),yaw_axis_world_candidate=axis.tolist(),
        moving_motor_ids=(moving+1).tolist(),sample_period_s=.05,nominal_suffix_seconds=SAMPLES*.05,
        maximum_delta_limit_ticks=MAX_DELTA,selected=record,attempts=attempts,model_manifest=manifest,
        operator_report='Cube felt caught by fingers while being removed; some movement may have occurred during removal.',
        operator_report_is_rotation_success=False,marker_to_model_registration_verified=False,
        native_limits_sha256=hashlib.sha256((OUT/'native_motor_limits.json').read_bytes()).hexdigest(),
        current_or_PID_changed=False,joint_offsets_fitted=False,live_sac=False,
        hardware_execution_allowed=False,full_native_motion_screen_complete=False,
        physical_cube_rotation_verified=False,
        limitations=['This is a small coordinated kinematic target, not a learned online camera policy.',
            'The conditional marker mount and inferred cube pose are not calibrated contact geometry.',
            'Actual yaw must be judged before REMOVE, after the fixed after-motion observation.',
            'Full path and start screening are required separately before any operator run.'])
    path=OUT/'same_grasp_small_yaw_candidate.json'
    if path.exists():
        old=path.read_bytes();archive=OUT/('yaw_candidate_before_'+hashlib.sha256(old).hexdigest()[:16]+'.json')
        if not archive.exists():archive.write_bytes(old)
    path.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('CANDIDATE',path,'SHA256',hashlib.sha256(path.read_bytes()).hexdigest(),flush=True)


if __name__=='__main__':main()
