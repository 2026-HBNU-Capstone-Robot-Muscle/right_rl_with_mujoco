import json
import numpy as np
import mujoco
from mujoco_playground import registry
from final_pose_temporary_support_20261116 import build
from final_pose_start_scope_20261116 import correct_model
from final_grip_pose_plan_20261116 import OUT,REQUEST,FinalGripPosePlan
from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK,ticks_to_sim
from loaded_quarter_reentry_20261025 import limits
from new_pair_grasp_plan_20261027 import ENTRY
p=FinalGripPosePlan(REQUEST);raw=p.source_position();cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
m,_=build(env,OUT/'range_diagnostic.xml',book_projection_mm=-15);policy=correct_model(m,raw);m.opt.timestep=.005
mask=m.geom_bodyid==m.body('cube').id;m.geom_contype[mask]=0;m.geom_conaffinity[mask]=0
m.actuator_gainprm[:,0]*=.5;m.actuator_biasprm[:,1]*=.5
lo,hi=limits();simids=np.asarray(SIM_TO_MOTOR_IDS)-1;low=(lo[simids]-2048)*TICK;high=(hi[simids]-2048)*TICK
d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(raw);d.ctrl[:]=d.qpos[:16];d.qpos[-7:]=[0,0,2,1,0,0,0];mujoco.mj_forward(m,d)
worst={};scheduled=0.;steps=0
for phase,q,duration in [('initial_hold',raw,.5)]+[(s,q,.075) for s,q in p.plan(raw)]:
    d.ctrl[:]=ticks_to_sim(q);scheduled+=duration;target=round(scheduled/.005)
    for _ in range(target-steps):
        mujoco.mj_step(m,d);ex=np.maximum(low-d.qpos[:16],d.qpos[:16]-high)
        if phase in ('initial_hold',ENTRY):
            for j in (7,11,15):
                sim=int(np.flatnonzero(simids==j)[0])
                if raw[j]<lo[j]:ex[sim]=max((raw[j]-4-2048)*TICK-d.qpos[sim]-TICK,d.qpos[sim]-high[sim])
            sim=int(np.flatnonzero(simids==3)[0]);ex[sim]=max((policy['extended_lower_ticks']-2048)*TICK-d.qpos[sim]-TICK,d.qpos[sim]-high[sim])
        j=int(np.argmax(ex))
        if ex[j]>worst.get('excess_rad',0):worst=dict(id=int(simids[j]+1),phase=phase,excess_rad=float(ex[j]),actual_ticks=float(d.qpos[j]/TICK+2048),goal_ticks=int(q[simids[j]]))
    steps=target
print(json.dumps(worst))
