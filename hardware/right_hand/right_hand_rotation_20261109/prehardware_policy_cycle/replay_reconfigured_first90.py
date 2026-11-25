"""Initial placement -> original first90 -> changed hand posture -> clear hold."""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,save_json,sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'initial_to_reconfigured_first90'
    if folder.exists():raise RuntimeError('Preserve prior reconfigured first replay')
    first=base/'initial_to_reached_workstation90';shape=base/'reached_first90_reverse_quarter_probe';model,initial,_,_,provenance=assets('active_ccd50');d=restore(model,initial);g=Guard(model);cube=model.geom('cube').id
    a=np.load(first/'research_goals.npz',allow_pickle=False)['goals'];b=np.load(shape/'research_goals.npz',allow_pickle=False)['goals'];assert len(a)==3015
    parts=[('first90_3015',a,first/'end_state.npz'),('same_station_hand_posture336',b[:336],shape/'A_posture_alignment_state.npz'),('first90_clear_hold54',b[336:390],shape/'first90_space_hold_state.npz')]
    previous=motor(d.ctrl);commands=[];trajectory=[d.qpos.copy()];seams=[];cert=SweepCertificate(model,g.hand,.3);count=supported=certified=0;minimum=float('inf');Cref=np.load(first/'end_state.npz',allow_pickle=False)['qpos'][-7:-4];holdC=holdB=None;shift=drift=0.
    folder.mkdir();(folder/'source_replay_reconfigured_first90.py').write_bytes(Path(__file__).read_bytes())
    for name,goals,expectedfile in parts:
        for command in goals:
            assert np.max(abs(command-previous))<=2+1e-9 and np.all(command>=g.lo) and np.all(command<=g.hi);d.ctrl[:]=ticks_to_sim(command);previous=command.copy();commands.append(command.copy())
            for _ in range(75):
                mujoco.mj_step(model,d);assert not g.check(d),g.last
                assert np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])<=.08
                if name=='first90_clear_hold54':
                    C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3)
                    if holdC is None:holdC=C.copy();holdB=B.copy()
                    count+=1;supported+=sample_model(model,d,g.hand,cube)['supported_sample'];lower,bad=cert.query(d,C,B);certified+=not bad and lower>.3;minimum=min(minimum,lower);shift=max(shift,float(np.linalg.norm(C-holdC)*1000));drift=max(drift,orientation_error_deg(B,holdB))
            trajectory.append(d.qpos.copy())
        expected=np.load(expectedfile,allow_pickle=False);errors={k:float(np.max(abs(getattr(d,k)-expected[k]))) if expected[k].size else 0. for k in BOUND};assert max(errors.values())==0.,(name,errors)
        seams.append(dict(segment=name,goals=len(goals),after_goal=len(commands),bound_state_errors=errors));print(json.dumps(dict(segment=name,goals=len(commands),exact_bound_match=True)),flush=True)
    target=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],90.);error=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),target);tilt=face_up_error_deg(d.geom_xmat[cube].reshape(3,3),5,initial['up']);center=float(np.linalg.norm(d.geom_xpos[cube]-Cref)*1000)
    passed=bool(count==certified==4050 and supported/count>.8 and error<5 and tilt<5 and center<2 and shift<2 and drift<3)
    np.savez_compressed(folder/'research_goals.npz',start=motor(initial['ctrl']),goals=commands);np.savez_compressed(folder/'trajectory.npz',qpos=trajectory,control_seconds=.075);np.savez_compressed(folder/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
    sources=[first/'research_goals.npz',first/'end_state.npz',shape/'research_goals.npz',shape/'A_posture_alignment_state.npz',shape/'first90_space_hold_state.npz']
    r=dict(status='initial_to_reconfigured_first90_replayed',model_provenance=provenance,source_files_sha256={p.relative_to(OUT).as_posix():sha(p) for p in sources},steps=len(commands),seams=seams,first90_reconfigured_whole_hand_hold_passed=passed,actual_state_resets_after_initial_placement=0,
        hold_samples=count,upper_certified_samples=certified,minimum_upper_lower_mm=minimum,whole_hand_support_fraction=supported/max(1,count),final90_error_deg=error,tilt_deg=tilt,reference_A_center_error_mm=center,hold_shift_mm=shift,hold_drift_deg=drift,
        finger_identity_is_not_a_global_support_requirement=True,old_canonical_task_not_reclassified=True,next90_not_certified=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False,worst=g.worst)
    save_json(folder/'review.json',r);print(json.dumps({k:r[k] for k in ('steps','first90_reconfigured_whole_hand_hold_passed','final90_error_deg','reference_A_center_error_mm','minimum_upper_lower_mm')}),flush=True)


if __name__=='__main__':main()
