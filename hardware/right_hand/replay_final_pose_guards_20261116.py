"""Replay unchanged hardware held/progress guards on the actual model route."""
import json
import numpy as np
import mujoco
from mujoco_playground import registry
from final_pose_temporary_support_20261116 import build
from final_pose_start_scope_20261116 import correct_model
from held_joint_guard import HeldJointGuard
from goal_aware_progress_guard_20261025 import GoalAwareProgress
from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK,ticks_to_sim
from final_grip_pose_plan_20261116 import OUT,REQUEST,FinalGripPosePlan,sha,CANDIDATE

p=FinalGripPosePlan(REQUEST);raw=p.source_position();rows=p.plan(raw)
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
m,_=build(env,OUT/'guard_scene.xml',book_projection_mm=-10);correct_model(m,raw);m.opt.timestep=.005
mask=m.geom_bodyid==m.body('cube').id;m.geom_contype[mask]=0;m.geom_conaffinity[mask]=0
gain=m.actuator_gainprm.copy();bias=m.actuator_biasprm.copy();reports=[]
def actual(d):
    q=np.empty(16,dtype=int);q[np.asarray(SIM_TO_MOTOR_IDS)-1]=np.rint(d.qpos[:16]/TICK+2048).astype(int)
    return q
for scope,scale in [('whole_path',.5),('whole_path',1.),('whole_path',1.5),('new_pose_segment',.5)]:
    m.actuator_gainprm[:]=gain;m.actuator_biasprm[:]=bias
    m.actuator_gainprm[:,0]*=scale;m.actuator_biasprm[:,1]*=scale
    selected=rows
    start=raw
    if scope=='new_pose_segment':
        from final_grip_pose_plan_20261116 import candidate,MOVE,HOLD
        z=candidate();start=z['start'];selected=[(str(p),q) for p,q in zip(z['phases'],z['goals'])]
    d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(start);d.ctrl[:]=d.qpos[:16];d.qpos[-7:]=[0,0,2,1,0,0,0];mujoco.mj_forward(m,d)
    held=HeldJointGuard(start,start);progress=GoalAwareProgress();command=start.copy();fault=None
    for i,(phase,target) in enumerate(selected+[('stage_2_fixed_hold',selected[-1][1])]*20+[('final_grip_user_hold',selected[-1][1])]*400):
        measured=actual(d)
        try:
            held.check(measured,command,p.frozen_for_phase(phase),phase)
            progress.check(phase,p.progress_for_phase(phase),command,measured,i*.075)
        except RuntimeError as exc:
            fault=str(exc);break
        held.advance(measured,command,target);command=target.copy();d.ctrl[:]=ticks_to_sim(command)
        for _ in range(15):mujoco.mj_step(m,d)
    reports.append(dict(scope=scope,gain=scale,passed=fault is None,done=i+1,expected=len(selected)+420,fault=fault))
result=dict(passed=all(r['passed'] for r in reports),cases=reports,devices_opened=False,
    candidate_sha256=sha(CANDIDATE),held_policy=HeldJointGuard.VERSION,
    additional_fixed_hold_seconds=30,thresholds_changed=False)
(OUT/'runtime_guard_review.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
