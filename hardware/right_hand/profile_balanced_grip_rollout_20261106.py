"""Check compiled free-cube rollout against direct stepping and time a proposal."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import json,time
import numpy as np,mujoco
from mujoco import rollout
from dynamic_common_grip_search_20261105 import ROOT
from research_rotation_body_hold_20261104 import restore
from hand_motor_mapping import TICK,SIM_TO_MOTOR_IDS

def main():
    source=ROOT/'countergrip_level/seed0_source1_lift0_normal2.5_n900_resume_lower_staged_freebase/end_state.npz';m=mujoco.MjModel.from_binary_path(str(ROOT/'torque_cap_sensitivity/cap0.15/model.mjb'));z=np.load(source);d=restore(m,z);spec=mujoco.mjtState.mjSTATE_FULLPHYSICS;initial=np.zeros(mujoco.mj_stateSize(m,spec));mujoco.mj_getState(m,d,initial,spec);raw=np.load(ROOT/'lower_countergrip_station_force2_span12_equilibrium/seed4/direct_equilibrium.npz')['goals'];controls=np.repeat((raw[:,np.asarray(SIM_TO_MOTOR_IDS)-1]-2048)*TICK,75,axis=0);began=time.perf_counter();states,_=rollout.rollout(m,mujoco.MjData(m),initial,controls[:750],initial_warmstart=z['qacc_warmstart']);seconds=time.perf_counter()-began
    for ctrl in controls[:750]:d.ctrl[:]=ctrl;mujoco.mj_step(m,d)
    error=float(np.max(abs(states[0,-1,1:1+m.nq]-d.qpos)));assert error<1e-10,error
    began=time.perf_counter();states,_=rollout.rollout(m,mujoco.MjData(m),initial,controls,initial_warmstart=z['qacc_warmstart']);elapsed=time.perf_counter()-began;print(json.dumps(dict(short_rollout_seconds=seconds,short_rollout_qpos_error=error,full_rollout_steps=len(controls),full_rollout_seconds=elapsed,final_cube_position=states[0,-1,1+16:1+19].tolist(),devices_opened=False)))
if __name__=='__main__':main()
