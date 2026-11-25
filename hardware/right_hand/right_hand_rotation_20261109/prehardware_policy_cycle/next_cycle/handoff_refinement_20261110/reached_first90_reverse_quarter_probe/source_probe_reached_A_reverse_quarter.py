"""Actual first90 endpoint -> compatible A posture -> reverse quarter -> hold.

This explicitly requests the opposite next direction (global90 to0), not two
positive quarters. No cube pose or other dynamic state is reset between stages.
"""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,save_json,sha,angle
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'reached_first90_reverse_quarter_probe'
    if folder.exists():raise RuntimeError('Preserve prior reverse-quarter probe')
    source=base/'initial_to_reached_workstation90/end_state.npz';known=base/'resumed_next90_joined_branch';ref=np.load(known/'research_goals.npz',allow_pickle=False)
    target_end=np.load(known/'end_state.npz',allow_pickle=False);Bstart=np.load(base/'opposing_workspace_next90/start_state.npz',allow_pickle=False)
    model,initial,_,_,provenance=assets('active_ccd50');z=dict(np.load(source,allow_pickle=False));d=restore(model,z);g=Guard(model);cube=model.geom('cube').id;C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy()
    folder.mkdir();(folder/'source_probe_reached_A_reverse_quarter.py').write_bytes(Path(__file__).read_bytes());commands=[];trajectory=[d.qpos.copy()];stages=[];stop=[];previous=motor(d.ctrl)
    def execute(command):
        nonlocal previous,stop
        assert np.max(abs(command-previous))<=2+1e-9 and np.all(command>=g.lo) and np.all(command<=g.hi)
        d.ctrl[:]=ticks_to_sim(command);commands.append(command.copy());previous=command.copy()
        for _ in range(75):
            mujoco.mj_step(model,d);stop=g.check(d)
            if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
            if stop:break
        trajectory.append(d.qpos.copy())
    def checkpoint(name,extra=None):
        np.savez_compressed(folder/(name+'_state.npz'),**{k:getattr(d,k) for k in BOUND},time=d.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
        r=dict(stage=name,after_goals=len(commands),stop=list(stop),whole_hand=sample_model(model,d,g.hand,cube));r.update(extra or {});stages.append(r)
        print(json.dumps({k:v for k,v in r.items() if k!='whole_hand'}),flush=True)
    def hold(name,yaw,station,require_fingers):
        nonlocal stop
        start=len(commands);cert=SweepCertificate(model,g.hand,.3);count=supported=certified=0;minimum=float('inf');loaded={f:0 for f in require_fingers};C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy();shift=drift=0.
        for _ in range(54):
            command=previous.copy();d.ctrl[:]=ticks_to_sim(command);commands.append(command.copy())
            for _ in range(75):
                mujoco.mj_step(model,d);stop=g.check(d);count+=1
                if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                load=sample_model(model,d,g.hand,cube);supported+=load['supported_sample'];lower,bad=cert.query(d,d.geom_xpos[cube],d.geom_xmat[cube].reshape(3,3));minimum=min(minimum,lower);certified+=not bad and lower>.3
                for f in loaded:
                    force=sum(v['normal_N'] for n,v in load['by_geom'].items() if (model.body(int(model.geom_bodyid[model.geom(n).id])).name or '').startswith(f+'_'));loaded[f]+=force>.1
                shift=max(shift,float(np.linalg.norm(d.geom_xpos[cube]-C)*1000));drift=max(drift,orientation_error_deg(d.geom_xmat[cube].reshape(3,3),B))
                if bad:stop=list(stop)+['upper_certificate_failed']
                if stop:break
            trajectory.append(d.qpos.copy())
            if stop:break
        target=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],yaw);error=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),target);center=float(np.linalg.norm(d.geom_xpos[cube]-station)*1000);tilt=face_up_error_deg(d.geom_xmat[cube].reshape(3,3),5,initial['up'])
        passed=bool(not stop and count==certified==4050 and supported/count>.8 and shift<2 and drift<3 and error<5 and center<2 and tilt<5 and all(n/count>.8 for n in loaded.values()))
        checkpoint(name,dict(samples=count,certified_samples=certified,upper_lower_mm=minimum,support_fraction=supported/max(1,count),loaded_fractions={f:n/max(1,count) for f,n in loaded.items()},hold_shift_mm=shift,hold_drift_deg=drift,error_deg=error,station_error_mm=center,passed=passed,begin_goal=start))
        return passed
    target=motor(target_end['ctrl']);start=previous.copy();count=max(1,int(np.ceil(np.max(abs(target-start))/2)))
    for step in range(count):
        execute(np.rint(start+(target-start)*(step+1)/count))
        if stop:break
    checkpoint('A_posture_alignment')
    first_hold=False;next_hold=False
    if not stop:first_hold=hold('first90_space_hold',90.,C0,[])
    if first_hold:
        reverse=np.vstack([ref['goals'][-2::-1],ref['start']])
        for command in reverse:
            execute(command)
            if stop:break
        checkpoint('reverse_quarter',dict(executed_reverse_goals=len(commands)-stages[-1]['after_goals']))
        if not stop:next_hold=hold('next0_loaded_hold',0.,Bstart['qpos'][-7:-4],['mf','th'])
    np.savez_compressed(folder/'research_goals.npz',start=motor(z['ctrl']),goals=commands);np.savez_compressed(folder/'trajectory.npz',qpos=trajectory,control_seconds=.075)
    np.savez_compressed(folder/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
    sources=[source,known/'research_goals.npz',known/'end_state.npz',base/'opposing_workspace_next90/start_state.npz']
    r=dict(status='actual_first90_to_opposite_next_quarter_probe',source=source.relative_to(OUT).as_posix(),source_files_sha256={p.relative_to(OUT).as_posix():sha(p) for p in sources},model_provenance=provenance,steps=len(commands),stop=stop,stages=stages,first90_posture_hold_passed=first_hold,opposite_next_quarter_and_hold_passed=next_hold,
        additional_yaw_deg=angle(d.geom_xmat[cube].reshape(3,3),B0,initial['up']),actual_state_resets_after_start=0,requested_next_direction='negative90_global90_to0',two_positive_quarters_not_claimed=True,initial_prefix_not_replayed_by_this_probe=True,
        new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False,worst=g.worst)
    save_json(folder/'review.json',r);print(json.dumps({k:r[k] for k in ('steps','stop','first90_posture_hold_passed','opposite_next_quarter_and_hold_passed','additional_yaw_deg')}),flush=True)


if __name__=='__main__':main()
