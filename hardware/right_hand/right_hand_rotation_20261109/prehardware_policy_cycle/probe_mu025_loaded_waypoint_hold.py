"""Test whether a transient loaded MF/TH waypoint can actually hold the cube."""
from pathlib import Path
import json
import numpy as np
import mujoco

from native_cycle import OUT, assets, restore, Guard, BOUND, motor, ticks_to_sim, save_json, sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import orientation_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    base = OUT / 'next_cycle/handoff_refinement_20261110'
    root = base / 'robust_cycle_preparation_20261111'
    first = root / 'low_friction_first_hold_state_v4/end_state.npz'
    parent = base / 'robust_mu025_next_turn_teacher_v1'
    folder = root / 'low_friction_loaded_waypoint_hold_v1'
    if folder.exists():
        raise RuntimeError('Preserve previous hold trial')
    waypoint=368
    row = next(r for r in json.loads((parent/'review.json').read_text(encoding='utf-8'))['trace'] if r['step']==waypoint)
    assert row['contact_finger_normals_N']['mf']>.1 and row['contact_finger_normals_N']['th']>.1
    m, initial, _, _, provenance = assets('active_ccd50')
    m.geom_friction[:, 0] = .25
    z = dict(np.load(first, allow_pickle=False))
    d = restore(m, z)
    guard = Guard(m)
    cube = m.geom('cube').id
    commands = np.load(parent/'research_goals.npz', allow_pickle=False)['goals'][:waypoint]
    previous = motor(d.ctrl)
    for index, command in enumerate(commands):
        assert np.max(abs(command-previous))<=2+1e-9
        assert np.all(command>=guard.lo) and np.all(command<=guard.hi)
        previous=command.copy();d.ctrl[:]=ticks_to_sim(command)
        for _ in range(75):
            mujoco.mj_step(m,d)
            reasons=guard.check(d)
            assert not reasons,(index,reasons)
            assert np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])<=.08
    expected=np.load(parent/'trajectory.npz',allow_pickle=False)['qpos'][waypoint]
    difference=float(np.max(abs(d.qpos-expected)))
    assert difference==0.
    C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy()
    cert=SweepCertificate(m,guard.hand,.3)
    count=supported=clear=mf=th=0;minimum=float('inf');shift=drift=0.;stop=[]
    for _ in range(54):
        d.ctrl[:]=ticks_to_sim(previous)
        for _ in range(75):
            mujoco.mj_step(m,d);stop=guard.check(d)
            C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3)
            count+=1;load=sample_model(m,d,guard.hand,cube)
            supported+=load['supported_sample']
            forces={finger:sum(v['normal_N'] for name,v in load['by_geom'].items() if (m.body(int(m.geom_bodyid[m.geom(name).id])).name or '').startswith(finger+'_')) for finger in ('mf','th')}
            mf+=forces['mf']>.1;th+=forces['th']>.1
            lower,bad=cert.query(d,C,B);minimum=min(minimum,lower);clear+=not bad and lower>.3
            shift=max(shift,float(np.linalg.norm(C-C0)*1000));drift=max(drift,orientation_error_deg(B,B0))
            if stop:break
        if stop:break
    folder.mkdir();(folder/'source_probe_mu025_loaded_waypoint_hold.py').write_bytes(Path(__file__).read_bytes())
    np.savez_compressed(folder/'end_state.npz',**{key:getattr(d,key) for key in BOUND},time=d.time)
    result=dict(status='low_friction_transient_loaded_waypoint_hold_probe',source=first.relative_to(OUT).as_posix(),source_sha256=sha(first),
        parent_goals=(parent/'research_goals.npz').relative_to(OUT).as_posix(),parent_goals_sha256=sha(parent/'research_goals.npz'),
        waypoint_step=waypoint,source_trajectory_qpos_difference=difference,model_provenance=provenance,sliding_friction_hypothesis=.25,
        hold_samples=count,supported_samples=supported,upper_certified_samples=clear,MF_loaded_samples=mf,TH_loaded_samples=th,
        minimum_upper_lower_mm=minimum,maximum_shift_mm=shift,maximum_drift_deg=drift,stop=stop,
        waypoint_loaded_hold_passed=bool(not stop and count==supported==clear==4050 and mf/4050>.8 and th/4050>.8 and shift<2 and drift<3),
        waypoint_is_not_second_rotation_completion=True,actual_state_resets_after_initial_placement=0,
        original_native_guards_and_joint_limits_preserved=True,devices_opened=0,hardware_execution_allowed=False)
    save_json(folder/'review.json',result)
    print(json.dumps({k:result[k] for k in ('hold_samples','supported_samples','upper_certified_samples','MF_loaded_samples','TH_loaded_samples','maximum_shift_mm','maximum_drift_deg','stop','waypoint_loaded_hold_passed')}),flush=True)


if __name__=='__main__':main()
