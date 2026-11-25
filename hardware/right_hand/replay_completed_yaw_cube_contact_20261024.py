"""Offline prescribed-hand replay with an active 55mm, 100g cube.

The measured hand trajectory is an imposed boundary condition. Cube placement,
friction and marker-to-model registration remain hypotheses, not calibration.
No policy training or hardware command is performed.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib
import json
from pathlib import Path
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'grasped_yaw_trial_20261023_234707_886202'
OUT=ROOT/'right_hand_rotation_20261024'
BASE='stage_4_fixed_grasp_observation';TURN='stage_5_grasped_small_yaw';AFTER='stage_6_yaw_fixed_observation'


def main():
    OUT.mkdir(exist_ok=True)
    blob=(FOLDER/'result.json').read_bytes();trial=json.loads(blob)
    paired=json.loads((ROOT/'slow_grasp_demo_20261023_182105_573825/on_paired_pose_review.json').read_bytes())
    hypotheses=[p for p in paired['hypotheses'] if p['source'].endswith('grid_diagnostic') and p['palm_branch']==0]
    c0=np.median([p['cube_center_world_m'] for p in hypotheses],axis=0)
    rotations=np.array([p['cube_rotation_world'] for p in hypotheses]);Rc=Rotation.from_matrix(rotations).mean().as_matrix()
    axis=Rc[:,2]
    hold=[s for s in trial['samples'] if s['phase']==BASE][-20:]
    actual=np.median([s['actual'] for s in hold],axis=0)
    path=[s for s in trial['samples'] if s['phase'] in (TURN,AFTER)]
    times=np.r_[0.,[s['t']-path[0]['t']+.05 for s in path]]
    trace=np.vstack([actual,[s['actual'] for s in path]])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg);m,manifest=build_with_middle_thumb_hulls(env)
    physics=set_cube_physics(m,side_m=.055,mass_kg=.1)
    cube=m.geom('cube').id;body=int(m.geom_bodyid[cube]);d=mujoco.MjData(m)
    cube_flags=dict(contype=int(m.geom_contype[cube]),conaffinity=int(m.geom_conaffinity[cube]))
    assert m.geom_contype[cube] or m.geom_conaffinity[cube]
    quat=Rotation.from_matrix(Rc).as_quat()[[3,0,1,2]]
    def initialize(center):
        mujoco.mj_resetData(m,d);d.qpos[:16]=ticks_to_sim(actual);d.qpos[-7:]=np.r_[center,quat]
        d.ctrl[:]=ticks_to_sim(actual);mujoco.mj_forward(m,d)
    def cube_contacts():
        out=[]
        for c in d.contact:
            if cube in (int(c.geom1),int(c.geom2)):
                other=int(c.geom2) if int(c.geom1)==cube else int(c.geom1)
                out.append(dict(other=m.geom(other).name,distance_mm=float(c.dist)*1000))
        return out
    placements=[]
    for shift_mm in np.arange(0.,16.01,.25):
        center=c0+axis*shift_mm/1000;initialize(center);contacts=cube_contacts()
        overlap=max((max(0.,-c['distance_mm']) for c in contacts),default=0.)
        placements.append(dict(shift_mm=float(shift_mm),max_overlap_mm=overlap,contacts=contacts))
    # A pose hypothesis may have planar marker depth error. Explore in-plane
    # uncertainty too; never alter the hand or relax the overlap cutoff.
    if min(p['max_overlap_mm'] for p in placements)>.15:
        best=sorted(placements,key=lambda p:p['max_overlap_mm'])[:3]
        for old in best:
            for x_mm in np.arange(-2.,2.01,.5):
                for y_mm in np.arange(-2.,2.01,.5):
                    center=c0+axis*old['shift_mm']/1000+(Rc[:,0]*x_mm+Rc[:,1]*y_mm)/1000
                    initialize(center);contacts=cube_contacts()
                    overlap=max((max(0.,-c['distance_mm']) for c in contacts),default=0.)
                    placements.append(dict(shift_mm=old['shift_mm'],in_plane_shift_mm=[float(x_mm),float(y_mm)],
                        max_overlap_mm=overlap,contacts=contacts))
    possible=[p for p in placements if p['max_overlap_mm']<=.15]
    if not possible:
        near=sorted(placements,key=lambda p:p['max_overlap_mm'])[:3]
        for old in near:
            x_mm,y_mm=old.get('in_plane_shift_mm',[0.,0.])
            for shift_mm in np.arange(old['shift_mm']-.2,old['shift_mm']+.201,.025):
                center=c0+axis*shift_mm/1000+(Rc[:,0]*x_mm+Rc[:,1]*y_mm)/1000
                initialize(center);contacts=cube_contacts()
                overlap=max((max(0.,-c['distance_mm']) for c in contacts),default=0.)
                placements.append(dict(shift_mm=float(shift_mm),in_plane_shift_mm=[x_mm,y_mm],
                    max_overlap_mm=overlap,contacts=contacts))
        possible=[p for p in placements if p['max_overlap_mm']<=.15]
    if not possible:
        rejected=dict(status='initial_placement_rejected',source=str(FOLDER.relative_to(ROOT)),
            cube_flags=cube_flags,gravity=m.opt.gravity.tolist(),old_center_m=c0.tolist(),
            cube_normal_world=axis.tolist(),placement_hypotheses=placements,
            hardware_execution_allowed=False,real_devices_opened=False)
        (OUT/'completed_yaw_active_cube_replay.json').write_text(json.dumps(rejected,indent=2),encoding='utf-8')
        print('REJECTED',json.dumps(sorted(placements,key=lambda p:p['max_overlap_mm'])[:3]),flush=True)
        raise RuntimeError('No low-overlap initial cube placement along provisional normal')
    selected=min(possible,key=lambda p:sum(v*v for v in p.get('in_plane_shift_mm',[0.,0.])))
    xy=selected.get('in_plane_shift_mm',[0.,0.])
    start=c0+axis*selected['shift_mm']/1000+(Rc[:,0]*xy[0]+Rc[:,1]*xy[1])/1000
    palm=m.body('palm').id;initialize(start)
    print('SETUP',json.dumps(dict(cube_flags=cube_flags,gravity=m.opt.gravity.tolist(),
        old_center_m=c0.tolist(),cube_normal_world=axis.tolist(),
        palm_rotation=d.xmat[palm].reshape(3,3).tolist(),selected=selected)),flush=True)
    runs=[]
    native_friction=m.geom_friction.copy()
    for friction in (.4,.8,1.2):
        m.geom_friction[:]=native_friction;m.geom_friction[cube,0]=friction;initialize(start)
        history=[]
        for step in range(round(2./m.opt.timestep)):
            d.qpos[:16]=ticks_to_sim(actual);d.qvel[:16]=0;d.ctrl[:]=ticks_to_sim(actual)
            mujoco.mj_step(m,d)
        settled=d.qpos[-7:].copy();initialR=d.xmat[body].reshape(3,3).copy()
        start_contacts=cube_contacts()
        elapsed=0.
        while elapsed<times[-1]:
            sample=np.array([np.interp(elapsed,times,trace[:,i]) for i in range(16)])
            nxt=np.array([np.interp(min(elapsed+m.opt.timestep,times[-1]),times,trace[:,i]) for i in range(16)])
            q=ticks_to_sim(sample);qn=ticks_to_sim(nxt)
            d.qpos[:16]=q;d.qvel[:16]=(qn-q)/m.opt.timestep;d.ctrl[:]=q
            mujoco.mj_step(m,d);elapsed+=m.opt.timestep
            if step%25==0:history.append(dict(t=elapsed,center=d.qpos[-7:-4].tolist(),rotation=d.qpos[-4:].tolist()))
            step+=1
        finalR=d.xmat[body].reshape(3,3).copy();rot=Rotation.from_matrix(initialR.T@finalR).as_rotvec()
        row=dict(cube_friction_hypothesis=friction,
            pre_to_post_settle_translation_mm=((settled[:3]-start)*1000).tolist(),
            settled_center_m=settled[:3].tolist(),settled_contacts=start_contacts,
            post_motion_translation_mm=((d.qpos[-7:-4]-settled[:3])*1000).tolist(),
            post_motion_rotation_vector_local_deg=np.degrees(rot).tolist(),
            finite=bool(np.isfinite(d.qpos).all()),history=history)
        runs.append(row);print('ACTIVE CUBE REPLAY',json.dumps({k:v for k,v in row.items() if k!='history'}),flush=True)
    out=dict(status='prescribed_measured_hand_active_cube_contact_diagnostic',
        source=str(FOLDER.relative_to(ROOT)),source_sha256=hashlib.sha256(blob).hexdigest(),
        cube_physics=physics,cube_collision_flags=cube_flags,placement_axis_shift_mm=selected['shift_mm'],
        placement_in_plane_shift_mm=xy,placement_center_m=start.tolist(),
        initial_placement_hypotheses=placements,runs=runs,
        original_marker_registration_verified=False,cube_friction_measured=False,
        imposed_hand_trajectory_not_actuator_or_force_prediction=True,
        simulation_fitted_to_success=False,physical_rotation_verified=False,
        hardware_execution_allowed=False,real_devices_opened=False,
        limitations=['Placement shifted only along provisional cube normal to avoid initial interpenetration.',
            'This shift is a diagnostic scenario, not a measured pose correction.',
            'Recorded hand motion is prescribed each step; contact forces on the hand are not predicted.',
            'Native collision primitives and unmeasured friction can disagree with real contacts.'])
    (OUT/'completed_yaw_active_cube_replay.json').write_text(json.dumps(out,indent=2),encoding='utf-8')


if __name__=='__main__':main()
