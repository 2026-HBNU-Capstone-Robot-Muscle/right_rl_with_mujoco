"""Rate-limited free-object arrival at a separately initialized opposed grip.

Alternative station has its own 2 mm target; original station failure is
reported alongside it, never relabeled. This is native model research only.
"""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,SIM_TO_MOTOR_IDS,save_json,sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'opposing_workspace_Z_arrivals'
    if folder.exists():raise RuntimeError('Preserve prior arrivals')
    source=base/'first90_RF_shape_before_adduction/RF_park_MF_then_RF_shape_then_adduct/phase0_end_state.npz'
    reference=base/'opposing_workspace_Z_seed122/free_hold_tests/force0p8/end_state.npz'
    z=dict(np.load(source,allow_pickle=False));ref=dict(np.load(reference,allow_pickle=False))
    model,initial,_,_,provenance=assets('active_ccd50');guard=Guard(model);cube=model.geom('cube').id
    target=motor(ref['ctrl']);station=ref['qpos'][-7:-4];canonical=np.load(OUT/'baseline_archived_zero/end_state.npz',allow_pickle=False)['qpos'][-7:-4]
    orientation=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],90.)
    if np.any(target<guard.lo) or np.any(target>guard.hi):raise RuntimeError('Native goal bound')
    sid=np.array(SIM_TO_MOTOR_IDS)-1;mf=sid[4:8];th=sid[12:16];all_ids=np.arange(16);other=np.setdiff1d(all_ids,np.r_[mf,th])
    orders=(('together',[all_ids]),('MF_then_TH',[mf,th,other]),('TH_then_MF',[th,mf,other]),('MF_abduct_then_shape',[mf[:1],np.r_[mf[1:],th],other]))
    folder.mkdir();(folder/'source_probe_opposing_workspace_arrival.py').write_bytes(Path(__file__).read_bytes());rows=[]
    for label,groups in orders:
        actual=restore(model,z);check=Guard(model);previous=motor(actual.ctrl);commands=[];stop=[];phaseends=[]
        for phase,group in enumerate(groups):
            start=previous.copy();end=start.copy();end[group]=target[group];count=max(1,int(np.ceil(np.max(abs(end-start))/2)))
            for index in range(count):
                goal=np.rint(start+(end-start)*(index+1)/count).astype(int)
                if np.max(abs(goal-previous))>2 or np.any(goal<check.lo) or np.any(goal>check.hi):raise RuntimeError('Command contract changed')
                actual.ctrl[:]=ticks_to_sim(goal);previous=goal;commands.append(goal.copy())
                for _ in range(75):
                    mujoco.mj_step(model,actual);stop=check.check(actual)
                    if np.linalg.norm(actual.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                    if stop:break
                if stop:break
            phaseends.append(dict(phase=phase,after_goal=len(commands),station_error_mm=float(np.linalg.norm(actual.geom_xpos[cube]-station)*1000)))
            if stop:break
        count=supported=mf_loaded=th_loaded=0;history=[];C0=actual.geom_xpos[cube].copy();B0=actual.geom_xmat[cube].reshape(3,3).copy();drift=shift=0.
        if not stop:
            for cycle in range(94):
                commands.append(previous.copy())
                for _ in range(75):
                    mujoco.mj_step(model,actual);stop=check.check(actual)
                    if np.linalg.norm(actual.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                    if cycle>=40:
                        load=sample_model(model,actual,check.hand,cube);count+=1;supported+=load['supported_sample'];history.append(actual.qpos.copy())
                        forces={name:sum(v['normal_N'] for g,v in load['by_geom'].items() if (model.body(int(model.geom_bodyid[model.geom(g).id])).name or '').startswith(name+'_')) for name in ('mf','th')}
                        mf_loaded+=forces['mf']>.1;th_loaded+=forces['th']>.1
                        shift=max(shift,float(np.linalg.norm(actual.geom_xpos[cube]-C0)*1000));drift=max(drift,orientation_error_deg(actual.geom_xmat[cube].reshape(3,3),B0))
                    if stop:break
                if stop:break
        station_error=float(np.linalg.norm(actual.geom_xpos[cube]-station)*1000);error=orientation_error_deg(actual.geom_xmat[cube].reshape(3,3),orientation)
        certificate=SweepCertificate(model,check.hand,.3);lower,bad=certificate.query(actual,actual.geom_xpos[cube],actual.geom_xmat[cube].reshape(3,3));certified=0
        if not stop and station_error<2 and error<5 and not bad and lower>.3:
            geometry=restore(model,z)
            for qpos in history:
                geometry.qpos[:]=qpos;mujoco.mj_forward(model,geometry);value,failed=certificate.query(geometry,geometry.geom_xpos[cube],geometry.geom_xmat[cube].reshape(3,3));lower=min(lower,value)
                if failed or value<=.3:break
                certified+=1
        passed=bool(not stop and count==4050 and supported/4050>.8 and mf_loaded/4050>.8 and th_loaded/4050>.8 and certified==4050 and station_error<2 and error<5 and drift<3 and shift<2)
        sub=folder/label;sub.mkdir();np.savez_compressed(sub/'research_goals.npz',start=motor(z['ctrl']),goals=commands)
        np.savez_compressed(sub/'end_state.npz',**{key:getattr(actual,key) for key in BOUND},time=actual.time,up=initial['up'],initial_cube_rotation=initial['initial_cube_rotation'],initial_cube_center=initial['initial_cube_center'])
        row=dict(order=label,steps=len(commands),stop=stop,hold_samples=count,whole_hand_support_fraction=supported/max(1,count),MF_load_fraction=mf_loaded/max(1,count),TH_load_fraction=th_loaded/max(1,count),
            alternative_station_error_mm=station_error,original_canonical_station_error_mm=float(np.linalg.norm(actual.geom_xpos[cube]-canonical)*1000),orientation_error_deg=error,
            certified_upper_samples=certified,minimum_upper_lower_mm=lower,hold_drift_deg=drift,hold_drift_mm=shift,alternative_station_arrival_and_hold_passed=passed,
            phaseends=phaseends,worst=check.worst,terminal_guard=check.last,terminal_support=sample_model(model,actual,check.hand,cube),path=sub.relative_to(OUT).as_posix())
        save_json(sub/'review.json',row);rows.append(row);print(json.dumps({key:row[key] for key in ('order','steps','stop','alternative_station_error_mm','orientation_error_deg','alternative_station_arrival_and_hold_passed')}),flush=True)
    save_json(folder/'review.json',dict(status='actual_arrivals_at_explicit_alternative_opposed_station',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),goal_reference=reference.relative_to(OUT).as_posix(),goal_reference_sha256=sha(reference),
        model_provenance=provenance,alternative_station_world_m=station.tolist(),original_canonical_station_world_m=canonical.tolist(),rows=rows,
        cube_free_in_every_actual_timestep=True,actual_state_resets_after_start=0,old_station_failures_not_reclassified=True,original_guard_thresholds_preserved=True,
        initial_placement_first_rotation_and_two_turn_cycle_passed=False,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False))


if __name__=='__main__':main()
