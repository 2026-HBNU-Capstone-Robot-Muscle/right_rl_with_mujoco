"""Active-cube diagnostic of a bounded old turn delta at the new grasp.

The new hand response comes from this completed trial. Cube poses/friction and
registration remain hypotheses. Placement is selected by initial geometry, not
by a desired motion outcome. Hand response is prescribed, not force validated.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib
import json
from pathlib import Path
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from book_support_model_20261024 import build_book
from sac_cube_model import set_cube_physics
from hand_motor_mapping import ticks_to_sim

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'book_cube_grasp_trial_20261024_212855_824047/result.json'
OUT=ROOT/'right_hand_rotation_20261024/book_grasp_yaw_diagnostic';OUT.mkdir(exist_ok=True)
blob=SOURCE.read_bytes();trial=json.loads(blob)
base=np.array(trial['grasp_goal_ticks'])
hold=[s for s in trial['samples'] if s['phase'] in ('stage_2_fixed_hold','grasp_observation')]
actual=np.median([s['actual'] for s in hold[-20:]],axis=0);residual=actual-base
old=json.loads((ROOT/'right_hand_rotation_20261024/contact_refined_small_yaw_candidate.json').read_bytes())
goals=np.array(old['goal_ticks'])-np.array(old['base_goal_ticks'])+base
if max(abs(np.diff(np.vstack([base,goals]),axis=0)).ravel())>2:raise ValueError('Step bound')
scenario=json.loads((ROOT/'right_hand_rotation_20261024/completed_yaw_active_cube_replay.json').read_bytes())
c0=np.array(scenario['placement_center_m'])
paired=json.loads((ROOT/'slow_grasp_demo_20261023_182105_573825/on_paired_pose_review.json').read_bytes())
rotations=[h['cube_rotation_world'] for h in paired['hypotheses'] if h['source'].endswith('grid_diagnostic') and h['palm_branch']==0]
Rc=Rotation.from_matrix(rotations).mean().as_matrix();quat=Rotation.from_matrix(Rc).as_quat()[[3,0,1,2]]
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
env=registry.load('LeapCubeFaceUp',config=cfg)
m,_=build_book(env,OUT/'scene.xml',book_projection_mm=-10.,book_ylo_mm=-500.,book_yhi_mm=500.,support_projection_mm=50.)
set_cube_physics(m,side_m=.055,mass_kg=.1)
cube=m.geom('cube').id;body=int(m.geom_bodyid[cube]);d=mujoco.MjData(m)
tips=[m.geom(n).id for n in ('if_tip','mf_tip','th_tip')]
palms=[g for g in range(m.ngeom) if m.geom(g).name.startswith('palm_collision')]
hand={m.body('palm').id}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
up=-m.opt.gravity/np.linalg.norm(m.opt.gravity)

def initialize(center,friction=.8):
    mujoco.mj_resetData(m,d);m.geom_friction[cube,0]=friction
    d.qpos[:16]=ticks_to_sim(actual);d.ctrl[:]=d.qpos[:16]
    d.qpos[-7:]=np.r_[center,quat];mujoco.mj_forward(m,d)

placements=[]
for dx in range(-12,13,2):
    for dy in range(-12,13,2):
        for dz in range(-4,23,2):
            center=c0+(dx*Rc[:,0]+dy*Rc[:,1]+dz*Rc[:,2])/1000
            initialize(center)
            overlap=max((max(0.,-float(c.dist)*1000) for c in d.contact
                if cube in (int(c.geom1),int(c.geom2))),default=0.)
            if overlap>.15:continue
            gaps=np.array([mujoco.mj_geomDistance(m,d,cube,g,1.,None)*1000 for g in tips])
            palm_gap=min(mujoco.mj_geomDistance(m,d,cube,g,1.,None)*1000 for g in palms)
            placements.append(dict(shift_mm=[dx,dy,dz],center_m=center.tolist(),overlap_mm=overlap,
                tip_gaps_mm=gaps.tolist(),palm_gap_mm=float(palm_gap),
                geometric_score=float(max(gaps)+.1*np.linalg.norm([dx,dy,dz]))))
print('PLACEMENTS',len(placements),flush=True)
if not placements:raise RuntimeError('No low-overlap initial cube hypothesis')
selected=min(placements,key=lambda p:p['geometric_score'])
center=np.array(selected['center_m'])
print('SELECTED_GEOMETRY',json.dumps(selected),flush=True)

def simulate(path,friction,offset):
    initialize(center+offset,friction)
    previous=base.astype(float)
    def step(goal,velocity=None):
        q=ticks_to_sim(goal+residual);d.qpos[:16]=q;d.qvel[:16]=0 if velocity is None else velocity;d.ctrl[:]=q
        mujoco.mj_step(m,d)
    for _ in range(round(2./m.opt.timestep)):step(base)
    start=d.qpos[-7:].copy();startR=d.xmat[body].reshape(3,3).copy()
    for goal in path:
        dq=(ticks_to_sim(goal+residual)-ticks_to_sim(previous+residual))/.05
        cycles=round(.05/m.opt.timestep)
        for k in range(cycles):step(previous+(goal-previous)*(k+1)/cycles,dq)
        previous=goal
    for _ in range(round(1./m.opt.timestep)):step(previous)
    rotation=Rotation.from_matrix(d.xmat[body].reshape(3,3)@startR.T).as_rotvec()
    shift=(d.qpos[-7:-4]-start[:3])*1000
    contacts=sorted({m.geom(int(c.geom2) if int(c.geom1)==cube else int(c.geom1)).name for c in d.contact
        if cube in (int(c.geom1),int(c.geom2)) and c.dist<.0003})
    return dict(yaw_deg=float(np.degrees(rotation@up)),tilt_deg=float(np.degrees(np.linalg.norm(rotation-up*(rotation@up)))),
        translation_mm=shift.tolist(),lateral_shift_mm=float(np.linalg.norm(shift-up*(shift@up))),
        settled_center_m=start[:3].tolist(),final_cube_contacts=contacts,finite=bool(np.isfinite(d.qpos).all()))

runs=[]
cases=[(.4,np.zeros(3)),(.8,np.zeros(3)),(1.2,np.zeros(3)),
    (.6,Rc[:,0]*.001),(1.,-Rc[:,0]*.001),(.6,Rc[:,1]*.001),(1.,-Rc[:,1]*.001)]
for friction,offset in cases:
    action=simulate(goals,friction,offset)
    baseline=simulate(np.tile(base,(len(goals),1)),friction,offset)
    row=dict(friction=friction,position_offset_m=offset.tolist(),action=action,no_action=baseline,
        action_minus_baseline_yaw_deg=action['yaw_deg']-baseline['yaw_deg'])
    runs.append(row);print('RUN',json.dumps(row),flush=True)
result=dict(source=str(SOURCE.relative_to(ROOT)),source_sha256=hashlib.sha256(blob).hexdigest(),
    base_goal_ticks=base.tolist(),measured_grasp_ticks=actual.tolist(),measured_response_residual_ticks=residual.tolist(),
    goal_ticks=goals.tolist(),selected_placement_hypothesis=selected,low_overlap_placement_hypotheses=placements,
    cube_rotation_world_hypothesis=Rc.tolist(),runs=runs,
    desired_physical_yaw_degrees=3.,old_candidate_sha256=hashlib.sha256((ROOT/'right_hand_rotation_20261024/contact_refined_small_yaw_candidate.json').read_bytes()).hexdigest(),
    max_goal_delta_ticks=int(np.max(abs(goals-base))),marker_to_model_registration_verified=False,
    prescribed_hand_response_hypothesis=True,physical_rotation_verified=False,hardware_execution_allowed=False,hardware_commands_sent=0)
(OUT/'relative_candidate_replay.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
