"""Recheck historical initialized RF hold endpoints on the current CCD50 model."""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,save_json,sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'RF_endpoint_CCD50_load_screen'
    if folder.exists():raise RuntimeError('Preserve previous RF endpoint recheck')
    m,initial,_,_,provenance=assets('active_ccd50');cube=m.geom('cube').id
    pool_path=base/'early_CCD50_states/state_pool.npz';pool=np.load(pool_path,allow_pickle=False)
    index=int(np.flatnonzero(pool['indices']==1700)[0]);checkpoint_center=pool['qpos'][index][-7:-4]
    target=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],90);rows=[];folder.mkdir()
    (folder/'source_revalidate_RF_endpoint_holds.py').write_bytes(Path(__file__).read_bytes())
    for source in sorted((base/'endpoint_screen').glob('qualified*/end_state.npz')):
        state=dict(np.load(source,allow_pickle=False));d=restore(m,state);guard=Guard(m);certificate=SweepCertificate(m,guard.hand,.3)
        B0=d.geom_xmat[cube].reshape(3,3).copy();C0=d.geom_xpos[cube].copy();supported=rf_loaded=0
        max_angle=max_shift=0.;gap=float('inf');bad=0;stop=[]
        for step in range(4050):
            mujoco.mj_step(m,d);stop=guard.check(d)
            if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
            support=sample_model(m,d,guard.hand,cube);supported+=support['supported_sample']
            rf_loaded+=sum(value['normal_N'] for name,value in support['by_geom'].items() if name.startswith('rf_'))>.2
            B=d.geom_xmat[cube].reshape(3,3);C=d.geom_xpos[cube]
            max_angle=max(max_angle,orientation_error_deg(B,B0));max_shift=max(max_shift,float(np.linalg.norm(C-C0)*1000))
            if stop:break
        hold=bool(not stop and step+1==4050 and supported/4050>.8 and rf_loaded/4050>.8 and max_angle<3 and max_shift<2)
        pose_error=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),target)
        tilt=face_up_error_deg(d.geom_xmat[cube].reshape(3,3),5,initial['up'])
        if hold and pose_error<5 and tilt<5:
            gap,violations=certificate.query(d,d.geom_xpos[cube],d.geom_xmat[cube].reshape(3,3));bad=int(bool(violations))
        row=dict(source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),steps_1ms=step+1,stop=stop,
            initialized_RF_hold_passed=hold,whole_hand_supported_fraction=supported/(step+1),RF_loaded_fraction=rf_loaded/(step+1),
            maximum_angle_drift_deg=max_angle,maximum_position_drift_mm=max_shift,
            final_global90_error_deg=pose_error,final_tilt_deg=tilt,upper_strict_min_gap_mm=gap if np.isfinite(gap) else None,
            upper_bad_samples=bad,initialized_pose_and_final_space_screen_passed=bool(hold and pose_error<5 and tilt<5 and not bad and gap>.3),
            initialized_work_grip_fully_certified=False,upper_certificate_scope='Final pose only, not the entire hold window',
            initial_cube_center=C0.tolist(),translation_from_checkpoint1700_mm=((C0-checkpoint_center)*1000).tolist(),
            translation_from_checkpoint1700_norm_mm=float(np.linalg.norm(C0-checkpoint_center)*1000),
            worst=guard.worst,terminal_support=sample_model(m,d,guard.hand,cube),certificate=certificate.metadata())
        rows.append(row)
        np.savez_compressed(folder/(source.parent.name+'_end_state.npz'),**{key:getattr(d,key) for key in BOUND},time=d.time,
            up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
        save_json(folder/'progress.json',dict(completed=rows,hardware_execution_allowed=False))
        print(json.dumps({key:row[key] for key in ('source','initialized_RF_hold_passed','initialized_pose_and_final_space_screen_passed',
            'final_global90_error_deg','upper_strict_min_gap_mm','translation_from_checkpoint1700_norm_mm','stop')}),flush=True)
    report=dict(status='historical_RF_initialized_holds_rechecked_on_CCD50',model_provenance=provenance,rows=rows,
        source_checkpoint_pool_sha256=sha(pool_path),initialized_endpoint_is_not_reached_trajectory=True,
        physical_or_full_cycle_success=False,devices_opened=0,hardware_execution_allowed=False)
    save_json(folder/'review.json',report)
    (folder/'source_revalidate_RF_endpoint_holds.py').write_bytes(Path(__file__).read_bytes())


if __name__=='__main__':main()
