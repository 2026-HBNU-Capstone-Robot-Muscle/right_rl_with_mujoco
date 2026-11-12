"""Offline small-turn candidates around a stationary ring-tip model point.

The last actual fixed-grasp readings describe actuator following error only.
No joint zero offsets or learned/live control are fitted. No device access.
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
SOURCE=ROOT/'grasped_yaw_trial_20261023_234707_886202/result.json'
AXIS_SOURCE=ROOT/'right_hand_rotation_20261023/same_grasp_small_yaw_candidate.json'
OUT=ROOT/'right_hand_rotation_20261023'
SAMPLES=90;MAX_DELTA=64


def main():
    raw=SOURCE.read_bytes();trial=json.loads(raw)
    assert trial['rotation_command_path_completed'] and trial['all_torque_off_verified'] and trial['settings_restored']
    assert trial['failed_phase']=='remove_cube' and not trial.get('read_failures')
    axis_blob=AXIS_SOURCE.read_bytes();old=json.loads(axis_blob)
    hold=[s for s in trial['samples'] if s['phase']=='stage_4_fixed_grasp_observation'][-20:]
    base=endpoints()[1];actual=np.median([s['actual'] for s in hold],axis=0);residual=actual-base
    axis=np.array(old['yaw_axis_world_candidate']);axis/=np.linalg.norm(axis)
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg);m,manifest=build_with_middle_thumb_hulls(env);d=mujoco.MjData(m)
    cube=m.geom('cube').id;m.geom_contype[cube]=m.geom_conaffinity[cube]=0
    support={m.geom(n).id for n in ('support_glass','support_wood')};hand={m.body('palm').id}
    for i in range(m.nbody):
        if int(m.body_parentid[i]) in hand:hand.add(i)
    names=('if_tip','mf_tip','th_tip');tip_ids=[m.geom(n).id for n in names]
    moving=np.array([0,1,2,4,5,6,8,9,10])
    limits_blob=(OUT/'native_motor_limits.json').read_bytes();limits=json.loads(limits_blob)
    lo=np.maximum(-MAX_DELTA,np.array(limits['lower_ticks'])[moving]-base[moving])
    hi=np.minimum(MAX_DELTA,np.array(limits['upper_ticks'])[moving]-base[moving])
    def pose(goal,offset):
        d.qpos[:16]=ticks_to_sim(goal+offset);d.qpos[-7:]=[0,0,2,1,0,0,0]
        d.qvel[:]=0;mujoco.mj_forward(m,d)
        return d.geom_xpos[tip_ids].copy()
    initial=pose(base,residual);pivot=d.geom_xpos[m.geom('rf_tip').id].copy()
    original_center=np.array(old['cube_center_world_m_candidate'])
    def overlaps(goals):
        self_max=support_max=0.
        for offset in (np.zeros(16),residual):
            for goal in goals:
                pose(goal,offset)
                for c in d.contact:
                    a,b=int(c.geom1),int(c.geom2)
                    ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
                    if ha and hb:self_max=max(self_max,-float(c.dist)*1000)
                    if (a in support and hb) or (b in support and ha):support_max=max(support_max,-float(c.dist)*1000)
        return self_max,support_max
    attempts=[];chosen=None
    for angle in (-5.,5.,-4.,4.,-3.,3.):
        goals=[];previous=np.zeros(len(moving));errors=[]
        for step in range(1,SAMPLES+1):
            u=step/SAMPLES;smooth=u*u*(3-2*u)
            rotation=Rotation.from_rotvec(axis*np.radians(angle)*smooth).as_matrix()
            target=(initial-pivot)@rotation.T+pivot
            def objective(x):
                q=base.astype(float).copy();q[moving]+=x
                return np.r_[(pose(q,residual)-target).ravel()*1000,.015*x]
            fit=least_squares(objective,previous,bounds=(lo,hi),diff_step=.025,
                ftol=1e-8,xtol=1e-7,gtol=1e-7,max_nfev=60)
            previous=fit.x;q=base.copy();q[moving]+=np.rint(previous).astype(int)
            goals.append(q);errors.append(np.linalg.norm(pose(q,residual)-target,axis=1)*1000)
        goals=np.array(goals);errors=np.array(errors);self_max,support_max=overlaps(goals)
        row=dict(target_yaw_degrees=angle,maximum_tip_target_error_mm=float(errors.max()),
            final_tip_target_error_mm=errors[-1].tolist(),maximum_goal_delta_ticks=int(np.max(abs(goals-base))),
            maximum_step_ticks=int(np.max(abs(np.diff(np.vstack([base,goals]),axis=0)))),
            raw_and_observed_response_static_self_overlap_mm=self_max,
            raw_and_observed_response_static_support_overlap_mm=support_max,
            moving_motor_delta_ticks=(goals[-1]-base).tolist())
        attempts.append(row);print('PIVOT CANDIDATE',json.dumps(row),flush=True)
        if (row['maximum_tip_target_error_mm']<=1.5 and row['maximum_step_ticks']<=2
                and self_max<=1e-6 and support_max<=1e-6):
            chosen=(row,goals);break
    if chosen is None:
        (OUT/'ring_pivot_candidate_search.json').write_text(json.dumps(dict(attempts=attempts,accepted=False,
            pivot_world_m=pivot.tolist(),initial_active_tip_centers_m=initial.tolist(),
            old_cube_center_m=original_center.tolist(),axis_world=axis.tolist(),
            initial_tip_pivot_distances_mm=(np.linalg.norm(initial-pivot,axis=1)*1000).tolist()),indent=2),encoding='utf-8')
        raise RuntimeError('No ring-pivot candidate passes existing bounds')
    row,goals=chosen
    result=dict(version='same_grasp_ring_pivot_yaw_candidate_v1',base_goal_ticks=base.tolist(),
        goal_ticks=goals.tolist(),source_trial=SOURCE.parent.name,source_trial_sha256=hashlib.sha256(raw).hexdigest(),
        source_baseline_phase='stage_4_fixed_grasp_observation',sampled_actual_ticks=actual.tolist(),
        recorded_following_residual_ticks=residual.tolist(),yaw_axis_world_candidate=axis.tolist(),
        pivot_geom='rf_tip',pivot_world_m_candidate=pivot.tolist(),
        previous_cube_center_world_m_candidate=original_center.tolist(),
        axis_source=str(AXIS_SOURCE.relative_to(ROOT)),axis_source_sha256=hashlib.sha256(axis_blob).hexdigest(),
        moving_motor_ids=(moving+1).tolist(),sample_period_s=.05,nominal_suffix_seconds=4.5,
        maximum_delta_limit_ticks=MAX_DELTA,selected=row,attempts=attempts,
        native_limits_sha256=hashlib.sha256(limits_blob).hexdigest(),model_manifest=manifest,
        pivot_is_measured_contact_point=False,marker_to_model_registration_verified=False,
        cube_dynamic_contact_simulated=False,current_or_PID_changed=False,joint_offsets_fitted=False,
        live_sac=False,hardware_execution_allowed=False,full_native_motion_screen_complete=False,
        physical_cube_rotation_verified=False,
        limitations=['The ring geom center is a provisional fixed pivot, not a measured cube contact.',
            'This is a kinematic small-motion test; cube friction/contact dynamics are not certified.',
            'The existing recorded following residual describes actuator response, not joint zero offsets.',
            'Actual cube direction must be reviewed before removal.'])
    path=OUT/'same_grasp_ring_pivot_yaw_candidate.json'
    if path.exists():
        previous=path.read_bytes();archive=OUT/('ring_pivot_candidate_before_'+hashlib.sha256(previous).hexdigest()[:16]+'.json')
        if not archive.exists():archive.write_bytes(previous)
    path.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('CANDIDATE',str(path),'SHA256',hashlib.sha256(path.read_bytes()).hexdigest(),flush=True)


if __name__=='__main__':main()
