"""Reproduce the cold model settling and subsequent thumb-only closure exactly.

The cold placement and initial control assignment are explicit. They are not a
physical arrival path, and the old9.111mm placement movement remains a failure.
"""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,save_json,sha
from whole_hand_support import sample_model
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'ring_thumb_closure_replay'
    if folder.exists():raise RuntimeError('Preserve earlier replay')
    geometry=base/'ring_thumb_photo_prior_seed431/axis0_ring+1';proposal=geometry/'proposal.npz';settled=geometry/'free_hold_tests/force0p8/end_state.npz';closure=base/'ring_thumb_only_closure/custom_settled_grip_extra0p8'
    z=dict(np.load(proposal,allow_pickle=False));expected=dict(np.load(settled,allow_pickle=False));final=dict(np.load(closure/'end_state.npz',allow_pickle=False));goals=np.load(closure/'research_goals.npz',allow_pickle=False)['goals']
    model,initial,_,_,provenance=assets('active_ccd50');d=restore(model,z);d.qvel[:]=0;d.qacc_warmstart[:]=0;d.ctrl[:]=expected['ctrl'];mujoco.mj_forward(model,d)
    g=Guard(model);assert not g.check(d);cube=model.geom('cube').id;folder.mkdir();(folder/'source_replay_ring_thumb_closure.py').write_bytes(Path(__file__).read_bytes());seams=[]
    for _ in range(7050):
        mujoco.mj_step(model,d);assert not g.check(d),g.last
        assert np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])<=.08
    errors={k:float(np.max(abs(getattr(d,k)-expected[k]))) if expected[k].size else 0. for k in BOUND};assert max(errors.values())==0.,errors;seams.append(dict(stage='cold_settle7050ms',errors=errors))
    fixed=d.ctrl[:12].copy();previous=motor(d.ctrl);cert=SweepCertificate(model,g.hand,.3);count=clear=supported=rf=th=0;minimum=float('inf')
    for i,command in enumerate(goals):
        assert np.max(abs(command-previous))<=2+1e-9 and np.all(command>=g.lo) and np.all(command<=g.hi)
        d.ctrl[12:16]=ticks_to_sim(command)[12:16];previous=command.copy();assert np.array_equal(d.ctrl[:12],fixed)
        for _ in range(75):
            mujoco.mj_step(model,d);assert not g.check(d),g.last;assert np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])<=.08
            if i>=len(goals)-54:
                count+=1;load=sample_model(model,d,g.hand,cube);supported+=load['supported_sample'];forces={name:sum(v['normal_N'] for geom,v in load['by_geom'].items() if (model.body(int(model.geom_bodyid[model.geom(geom).id])).name or '').startswith(name+'_')) for name in ('rf','th')}
                rf+=forces['rf']>.1;th+=forces['th']>.1;lower,bad=cert.query(d,d.geom_xpos[cube],d.geom_xmat[cube].reshape(3,3));minimum=min(minimum,lower);clear+=not bad and lower>.3
    errors={k:float(np.max(abs(getattr(d,k)-final[k]))) if final[k].size else 0. for k in BOUND};assert max(errors.values())==0.,errors;seams.append(dict(stage='thumb_only'+str(len(goals))+'goals',errors=errors))
    assert count==clear==supported==rf==th==4050
    report=dict(status='passed',source_files_sha256={p.relative_to(OUT).as_posix():sha(p) for p in (proposal,settled,closure/'research_goals.npz',closure/'end_state.npz')},model_provenance=provenance,
        seams=seams,thumb_closure_goals=len(goals),hold_samples=count,upper_certified_samples=clear,RF_loaded_samples=rf,TH_loaded_samples=th,whole_hand_supported_samples=supported,upper_minimum_lower_mm=minimum,
        non_thumb_commands_bitwise_unchanged=True,cold_initial_pose_and_control_assignment_explicit=True,old_placement_shift_failure_preserved=True,
        actual_state_resets_after_cold_start=0,physical_photo_not_registered=True,arrival_from_first_rotation_not_certified=True,full_two_turn_cycle_passed=False,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False)
    save_json(folder/'review.json',report);print(json.dumps(dict(status='passed',seams=seams,loaded_samples=rf,certified_samples=clear,minimum_upper_mm=minimum)),flush=True)


if __name__=='__main__':main()
