"""Fit local bounded command-to-object responses around the nominal first turn.

This is finite-difference model identification, not SAC training or policy
distillation. Native geometry and guards are unchanged for every branch.
"""
from pathlib import Path
import json
import time
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,save_json,sha


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';root=base/'robust_cycle_preparation_20261111';folder=root/'local_pose_response'
    if folder.exists():raise RuntimeError('Preserve pose-response fit')
    pool_file=root/'failure_checkpoints/nominal/state_pool.npz';pool=np.load(pool_file,allow_pickle=False);ref_file=base/'initial_second_turn_hold_replay/research_goals.npz';goals=np.load(ref_file,allow_pickle=False)['goals'];poses=np.load(ref_file.parent/'trajectory.npz',allow_pickle=False)['qpos']
    m,initial,_,_,provenance=assets('active_ccd50');cube=m.geom('cube').id;horizon=6;amplitude=4.;phases=[];jacobians=[];gains=[];valid_counts=[];began=time.perf_counter();max_nominal_error=0.
    folder.mkdir();(folder/'source_fit_cycle_pose_response.py').write_bytes(Path(__file__).read_bytes())
    for pos,phase in enumerate(pool['indices']):
        phase=int(phase)
        if not 1700<=phase<=2420 or phase%20:continue
        state=dict(initial,**{k:pool[k][pos].copy() for k in BOUND});state['time']=float(pool['time'][pos])
        def predict(bias):
            d=restore(m,state);g=Guard(m);g.gap,g.longest_gap=pool['guard_gap'][pos];previous=motor(d.ctrl)
            for step in range(horizon):
                requested=np.clip(goals[phase+step]+bias,np.ceil(g.lo),np.floor(g.hi));command=previous+np.clip(requested-previous,-2,2);d.ctrl[:]=ticks_to_sim(command);previous=command
                for _ in range(75):
                    mujoco.mj_step(m,d)
                    if g.check(d) or np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:return None
            return d
        zero=predict(np.zeros(16));assert zero is not None,phase
        delta=float(np.max(abs(zero.qpos-poses[phase+horizon])));max_nominal_error=max(max_nominal_error,delta);assert delta==0.,(phase,delta)
        C=zero.geom_xpos[cube].copy();B=zero.geom_xmat[cube].reshape(3,3).copy()
        def response(d):return np.r_[(d.geom_xpos[cube]-C)*1000,Rotation.from_matrix(d.geom_xmat[cube].reshape(3,3)@B.T).as_rotvec()*180/np.pi*.35]
        J=np.zeros((6,16));valid=0
        for joint in range(16):
            ends=[]
            for sign in (-1,1):
                bias=np.zeros(16);bias[joint]=sign*amplitude;d=predict(bias);ends.append(None if d is None else response(d));valid+=d is not None
            if all(v is not None for v in ends):J[:,joint]=(ends[1]-ends[0])/(2*amplitude)
            elif ends[0] is not None:J[:,joint]=-ends[0]/amplitude
            elif ends[1] is not None:J[:,joint]=ends[1]/amplitude
        # Millimetre and weighted-degree error maps to an absolute residual
        # in motor ticks. The independent runtime limits each executed delta.
        K=J.T@np.linalg.solve(J@J.T+.025**2*np.eye(6),np.eye(6))
        phases.append(phase);jacobians.append(J);gains.append(K);valid_counts.append(valid)
        if len(phases)%6==0:print(json.dumps(dict(phases=len(phases),reference_phase=phase,elapsed_seconds=time.perf_counter()-began)),flush=True)
    target=folder/'response.npz';np.savez_compressed(target,phases=phases,jacobians=jacobians,gains=gains,valid_probe_counts=valid_counts,orientation_weight=.35,reference_horizon=horizon,bias_probe_ticks=amplitude)
    save_json(folder/'review.json',dict(status='bounded_local_model_pose_response_fit',source_pool=pool_file.relative_to(OUT).as_posix(),source_pool_sha256=sha(pool_file),reference=ref_file.relative_to(OUT).as_posix(),reference_sha256=sha(ref_file),response=target.relative_to(OUT).as_posix(),response_sha256=sha(target),model_provenance=provenance,phases=len(phases),phase_range=[phases[0],phases[-1]],horizon_cycles=horizon,amplitude_ticks=amplitude,nominal_branch_maximum_qpos_difference=max_nominal_error,valid_probe_counts=valid_counts,only_native_admissible_branches_used=True,actual_closed_loop_rollout_not_yet_evaluated=True,finite_difference_fit_is_not_SAC_or_distillation=True,source_guard_thresholds_not_changed=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False));print(json.dumps(dict(phases=len(phases),minimum_valid_probes=min(valid_counts),maximum_valid_probes=max(valid_counts))),flush=True)


if __name__=='__main__':main()
