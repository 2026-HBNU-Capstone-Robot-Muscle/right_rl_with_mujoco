"""Assemble a no-output whole-cycle controller without modifying old bundles."""
from pathlib import Path
import hashlib
import json
import shutil
import numpy as np

OUT=Path(__file__).resolve().parent;BASE=OUT/'next_cycle/handoff_refinement_20261110';ROOT=OUT.parents[1]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    folder=BASE/'robust_cycle_preparation_20261111/shadow_bundle'
    if folder.exists():raise RuntimeError('Preserve prepared no-output bundle')
    folder.mkdir(parents=True);old=OUT/'next_cycle/staged_shadow_bundle_v2_ccd50';prefix=folder/'sac_prefix';prefix.mkdir()
    original_meta=json.loads((old/'bundle.json').read_text(encoding='utf-8'))
    for name in ('bundle.json',*original_meta['files_sha256']):
        target=prefix/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(old/name,target)
    nominal=BASE/'initial_second_turn_hold_replay';goals=np.load(nominal/'research_goals.npz',allow_pickle=False);poses=np.load(nominal/'trajectory.npz',allow_pickle=False);bounds=np.load(folder.parent/'command_contract.npz',allow_pickle=False)
    np.savez_compressed(folder/'reference.npz',start=goals['start'],goals=goals['goals'],qpos=poses['qpos'],lower=bounds['lower'],upper=bounds['upper'])
    shutil.copyfile(folder.parent/'encoder_clearance/kinematics.npz',folder/'kinematics.npz')
    for name in ('whole_cycle_shadow_runtime.py','cycle_joint_clearance.py','shadow_runtime.py','staged_shadow_runtime.py'):shutil.copyfile(OUT/name,folder/name)
    files={p.relative_to(folder).as_posix():sha(p) for p in folder.rglob('*') if p.is_file()}
    meta=dict(format='whole_cycle_no_output_v1',files_sha256=files,model_sha256='2608fe8475883c8250d8974956cc9209b50a0ca49c8bec6047970eec20b20b30',nominal_cycle_report_sha256=sha(nominal/'review.json'),first1700_use_existing_SAC=True,suffix_is_saved_model_teacher_not_SAC=True,MF_static_margin_mm=.15,phase_speed_mode='pose_adaptive',max_goal_ticks=2,control_seconds=.075,current_limit_mA=450,PID_changed=False,accepted_registration_id=None,physical_camera_registration_verified=False,whole_cycle_physical_validation_completed=False,model_force_is_not_calibrated_to450mA=True,old_7p4mm_station_miss_not_reclassified=True,final_hold_keeps_command=True,automatic_open=False,automatic_torque_off=False,source_execution_files_modified=0,devices_opened=0,hardware_execution_allowed=False)
    (folder/'bundle.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(dict(bundle=str(folder.relative_to(OUT)),files=len(files),hardware_execution_allowed=False)),flush=True)


if __name__=='__main__':main()
