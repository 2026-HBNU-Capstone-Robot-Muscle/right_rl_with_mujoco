"""Replay identical commands at converged collision iterations; save actual states."""
import argparse
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,save_json,sha
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg
from handoff_support_wrench import contact_wrench


def main():
    p=argparse.ArgumentParser();p.add_argument('--iterations',type=int,choices=(50,100),default=50);a=p.parse_args()
    folder=OUT/'next_cycle/handoff_refinement_20261110'/f'ccd_replay{a.iterations}';folder.mkdir(exist_ok=True)
    if (folder/'review.json').exists():raise RuntimeError('Preserve completed convergence replay')
    source=OUT/'next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_active_ccd50'
    recorded=source/'research_trace.npz';goals=np.load(recorded,allow_pickle=False)['goals']
    m,z,_,_,provenance=assets('active_ccd50');m.opt.ccd_iterations=a.iterations;d=restore(m,z);guard=Guard(m);cube=m.geom('cube').id
    target=upright_yaw_target(z['initial_cube_rotation'],z['up'],90);pool=[];metrics=[];stop=None
    checkpoints={0,3000,3400,3600,3800,3900,3960,4000,4040,4060,4080,4100}
    def capture(index):
        pool.append(dict(index=index,time=d.time,**{key:getattr(d,key).copy() for key in BOUND}))
        w=contact_wrench(m,d,guard.hand)
        metrics.append(dict(goal_index=index,target90_error_deg=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),target),
             cube_center=d.geom_xpos[cube].tolist(),forces_N_model_only={name:r['normal_N'] for name,r in w['by_geom'].items()}))
    capture(0)
    for i,goal in enumerate(goals):
        if np.max(abs(goal-motor(d.ctrl)))>2:raise AssertionError('Recorded command rate changed')
        d.ctrl[:]=ticks_to_sim(goal)
        for _ in range(75):
            mujoco.mj_step(m,d);stop=guard.check(d)
            if np.linalg.norm(d.qpos[-7:-4]-z['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
            if stop:break
        if i+1 in checkpoints:capture(i+1)
        if stop:break
    np.savez_compressed(folder/'state_pool.npz',indices=np.array([r['index'] for r in pool]),time=np.array([r['time'] for r in pool]),
                        **{key:np.array([r[key] for r in pool]) for key in BOUND})
    np.savez_compressed(folder/'end_state.npz',**{key:getattr(d,key) for key in BOUND},time=d.time,
        up=z['up'],initial_cube_rotation=z['initial_cube_rotation'],initial_cube_center=z['initial_cube_center'])
    expected=np.load(source/'end_state.npz',allow_pickle=False)
    diff={key:float(np.max(abs(getattr(d,key)-expected[key]))) if expected[key].size else 0. for key in BOUND}
    r=dict(status='same_command_CCD_dynamic_convergence_replay',iterations=a.iterations,source_model_provenance=provenance,
        runtime_option_override=dict(ccd_iterations=a.iterations),source_goals_sha256=sha(recorded),
        goals_completed=i+1,stop=stop,end_difference_from50=diff,checkpoints=metrics,
        exact_native_geometry_and_limits_unchanged=True,no_state_reset_during_replay=True,
        state_pool_is_actual_converged_model_states=True,state_pool_is_not_successful_training_demonstrations=True,
        previous_full_policy_passes_not_transferred=True,next90_rotation_passed=False,devices_opened=0,hardware_execution_allowed=False)
    save_json(folder/'review.json',r);print(json.dumps(r),flush=True)


if __name__=='__main__':main()
