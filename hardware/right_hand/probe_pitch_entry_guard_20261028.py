"""Check the existing measured CMC guard before the first root command."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,numpy as np,mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from loaded_quarter_model_policy_20261025 import correct_model
from loaded_quarter_reentry_20261025 import cmc_native_min
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS,TICK
from pitch_handoff_plan_20261028 import PitchHandoffPlan,REQUEST,OUT,ENTRY,ROOTS,prefix_rows
def main():
    p=PitchHandoffPlan(REQUEST);raw=p.source_position();rows=prefix_rows(p,raw)
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    m,_=build(registry.load('LeapCubeFaceUp',config=cfg),OUT/'entry_guard_scene.xml')
    correct_model(m,raw);m.opt.timestep=.005;gain=m.actuator_gainprm.copy();bias=m.actuator_biasprm.copy()
    j=list(SIM_TO_MOTOR_IDS).index(4);reports=[]
    for scale in (.5,1.,1.5):
        m.actuator_gainprm[:]=gain;m.actuator_biasprm[:]=bias
        m.actuator_gainprm[:,0]*=scale;m.actuator_biasprm[:,1]*=scale
        d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(raw);d.ctrl[:]=d.qpos[:16];d.qpos[-7:]=[0,0,2,1,0,0,0]
        for _ in range(100):mujoco.mj_step(m,d)
        steps=0
        for i,(phase,q) in enumerate(rows):
            if phase==ROOTS:
                actual=float(d.qpos[j]/TICK+2048);threshold=float(cmc_native_min()+4)
                reports.append(dict(gain_scale=scale,actual_motor4_ticks=actual,required_minimum_ticks=threshold,passed=actual>=threshold));break
            d.ctrl[:]=ticks_to_sim(q);target=round((i+1)*.075/.005)
            for _ in range(target-steps):mujoco.mj_step(m,d)
            steps=target
    report=dict(status='first_root_native_reentry_guard_diagnostic',cases=reports,all_passed=all(r['passed'] for r in reports),devices_opened=False)
    (OUT/'entry_guard_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report),flush=True)
if __name__=='__main__':main()
