"""Model-only PD goal compensation comparator; not SAC or a current calibration.

The model's physics, PID gains and native guards stay unchanged. The0.15Nm
number is an archived simulation scenario, never a conversion from450mA.
"""
import time
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,motor,ticks_to_sim,BOUND,save_json
from inspect_endpoints import inspect


def main():
    m,z,ref,_,provenance=assets('active');d=restore(m,z);guard=Guard(m)
    folder=OUT/'pd_goal_compensation_comparison';folder.mkdir(exist_ok=True)
    rows=[];previous=motor(d.ctrl);stop=None;max_force=0.;began=time.perf_counter()
    for index,nominal in enumerate(ref['goals']):
        desired=ticks_to_sim(nominal)
        gain=m.actuator_gainprm[:,0]
        bias=m.actuator_biasprm[:,0]+m.actuator_biasprm[:,1]*d.qpos[:16]+m.actuator_biasprm[:,2]*d.qvel[:16]
        predicted=gain*desired+bias
        shaped=(np.clip(predicted,-.15,.15)-bias)/gain
        proposal=motor(shaped);goal=previous+np.clip(proposal-previous,-2,2)
        if np.any(goal<guard.lo) or np.any(goal>guard.hi):stop=['native_goal_range'];break
        d.ctrl[:]=ticks_to_sim(goal);rows.append(goal);previous=goal
        for _ in range(75):
            mujoco.mj_step(m,d);stop=guard.check(d)
            max_force=max(max_force,float(np.max(np.abs(d.actuator_force))))
            if stop:break
        if (index+1)%500==0 or stop:
            print(json.dumps({'goals':index+1,'stop':stop,'seconds':time.perf_counter()-began}),flush=True)
        if stop:break
    np.savez_compressed(folder/'research_goals.npz',start=ref['start'],goals=rows)
    np.savez_compressed(folder/'end_state.npz',**{key:getattr(d,key) for key in BOUND},time=d.time,
                        up=z['up'],initial_cube_rotation=z['initial_cube_rotation'],initial_cube_center=z['initial_cube_center'])
    report={'status':'offline_PD_goal_compensation_comparison','model_provenance':provenance,
            'native_physics_and_force_limits_and_PID_unchanged':True,
            'model_PD_force_target_Nm':.15,'not_a_conversion_of450mA':True,
            'actual_max_model_actuator_force_Nm':max_force,'completed_goals':index+1,
            'stop':stop,'worst':guard.worst,'endpoint':inspect(m,d,z['up'],z['initial_cube_rotation']),
            'this_is_not_SAC_training_or_policy_success':True,
            'hardware_execution_allowed':False,'devices_opened':0,'elapsed_seconds':time.perf_counter()-began}
    save_json(folder/'review.json',report);print(json.dumps({k:v for k,v in report.items() if k not in ('endpoint','model_provenance')}),flush=True)


if __name__=='__main__':main()
