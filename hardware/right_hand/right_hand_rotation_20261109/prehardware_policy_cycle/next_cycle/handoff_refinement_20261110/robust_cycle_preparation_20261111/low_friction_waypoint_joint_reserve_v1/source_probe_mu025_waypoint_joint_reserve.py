"""Test small ID6 command reserves at an actual loaded intermediate waypoint."""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT, assets, restore, Guard, BOUND, motor, ticks_to_sim, save_json, sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import orientation_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';root=base/'robust_cycle_preparation_20261111'
    source=root/'low_friction_first_hold_state_v4/end_state.npz';parent=base/'robust_mu025_next_turn_teacher_v1'
    folder=root/'low_friction_waypoint_joint_reserve_v1'
    if folder.exists():raise RuntimeError('Preserve previous joint-reserve trial')
    goals=np.load(parent/'research_goals.npz',allow_pickle=False)['goals'][:368]
    expected=np.load(parent/'trajectory.npz',allow_pickle=False)['qpos'][368]
    folder.mkdir();(folder/'source_probe_mu025_waypoint_joint_reserve.py').write_bytes(Path(__file__).read_bytes());rows=[]
    for reserve in (0,1,2):
        m,initial,_,_,provenance=assets('active_ccd50');m.geom_friction[:,0]=.25
        z=dict(np.load(source,allow_pickle=False));d=restore(m,z);guard=Guard(m);cube=m.geom('cube').id
        previous=motor(d.ctrl)
        for i,command in enumerate(goals):
            assert np.max(abs(command-previous))<=2+1e-9 and np.all(command>=guard.lo) and np.all(command<=guard.hi)
            d.ctrl[:]=ticks_to_sim(command);previous=command.copy()
            for _ in range(75):
                mujoco.mj_step(m,d);assert not guard.check(d),(i,guard.last)
        assert np.array_equal(d.qpos,expected)
        command=previous.copy();command[5]-=reserve
        assert np.max(abs(command-previous))<=2 and np.all(command>=guard.lo) and np.all(command<=guard.hi)
        C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy();count=supported=mf=th=0;shift=drift=0.;stop=[];trajectory=[]
        for cycle in range(54):
            d.ctrl[:]=ticks_to_sim(command)
            for _ in range(75):
                mujoco.mj_step(m,d);stop=guard.check(d)
                if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3);load=sample_model(m,d,guard.hand,cube);count+=1;supported+=load['supported_sample']
                f={finger:sum(v['normal_N'] for name,v in load['by_geom'].items() if (m.body(int(m.geom_bodyid[m.geom(name).id])).name or '').startswith(finger+'_')) for finger in ('mf','th')}
                mf+=f['mf']>.1;th+=f['th']>.1
                shift=max(shift,float(np.linalg.norm(C-C0)*1000));drift=max(drift,orientation_error_deg(B,B0))
                if stop:break
            trajectory.append(d.qpos.copy())
            if stop:break
        sub=folder/f'ID6_reserve{reserve}tick';sub.mkdir()
        np.savez_compressed(sub/'end_state.npz',**{key:getattr(d,key) for key in BOUND},time=d.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
        np.savez_compressed(sub/'hold_goals.npz',start=previous,goals=np.tile(command,(cycle+1,1)))
        cert=SweepCertificate(m,guard.hand,.3);lower,bad=cert.query(d,d.geom_xpos[cube],d.geom_xmat[cube].reshape(3,3))
        stable=bool(not stop and count==4050 and supported/count>.8 and shift<2 and drift<3)
        result=dict(ID6_reserve_ticks=reserve,hold_samples=count,supported_samples=supported,MF_loaded_samples=mf,TH_loaded_samples=th,
            maximum_shift_mm=shift,maximum_drift_deg=drift,stop=stop,stable_support_hold_passed=stable,
            loaded_MF_TH_hold_passed=bool(stable and mf/count>.8 and th/count>.8),terminal_guard=guard.last,worst=guard.worst,
            final_upper_lower_bound_mm=lower,final_upper_uncertified_geoms=[m.geom(int(cert.ids[i])).name for i in bad],
            upper_test_scope='Final static diagnostic only; upper space is not required during intermediate rotation',
            path=sub.relative_to(OUT).as_posix(),devices_opened=0,hardware_execution_allowed=False)
        rows.append(result);save_json(sub/'review.json',result);print(json.dumps({k:result[k] for k in ('ID6_reserve_ticks','hold_samples','MF_loaded_samples','TH_loaded_samples','stop','stable_support_hold_passed','loaded_MF_TH_hold_passed')}),flush=True)
    save_json(folder/'review.json',dict(status='low_friction_loaded_waypoint_ID6_command_reserve_comparison',
        source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),parent_goals=(parent/'research_goals.npz').relative_to(OUT).as_posix(),parent_goals_sha256=sha(parent/'research_goals.npz'),
        source_waypoint_step=368,rows=rows,sliding_friction_hypothesis=.25,original_native_guards_and_joint_limits_preserved=True,
        actual_state_resets_after_first_hold=0,waypoint_is_not_second_rotation_completion=True,devices_opened=0,hardware_execution_allowed=False))


if __name__=='__main__':main()
