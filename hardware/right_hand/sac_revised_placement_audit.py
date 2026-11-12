"""Compare the newly photographed placement with the learned reset, offline."""
import json
from pathlib import Path
import numpy as np
from sac_registration_from_anchor import estimate
ROOT=Path(__file__).resolve().parent

def main():
    run=json.loads((ROOT/'leap_sac_start_capture_result.json').read_text(encoding='utf-8'))
    if run['status']!='revised_start_capture_complete' or not run['all_torque_off_verified']:
        raise RuntimeError('Capture did not complete with verified OFF')
    folder=Path(run['vision_recording']['folder'])
    reg=estimate(folder,empty_image='anchor_empty_preflight.png')
    reset=json.loads((ROOT/'sac_supported_reset_result.json').read_text())
    p=np.asarray(reg['cube_center_in_sim_world_candidate_m'])
    q=np.asarray(reg['cube_quat_world_candidate']);q/=np.linalg.norm(q)
    mapping=np.r_[np.arange(5,17),np.arange(1,5)]
    measured=np.asarray(run['placement_joint_ticks'])[mapping-1]*(2*np.pi/4096)-np.pi
    comparisons=[]
    for i,c in enumerate(reset['candidates']):
        qpos=np.asarray(c['qpos']);rq=qpos[-4:]/np.linalg.norm(qpos[-4:])
        joint_error=np.abs(measured-qpos[:16])
        comparisons.append(dict(offset_index=i,center_delta_mm=((p-qpos[-7:-4])*1000).tolist(),
            center_distance_mm=float(np.linalg.norm(p-qpos[-7:-4])*1000),
            orientation_error_deg=float(np.degrees(2*np.arccos(np.clip(abs(q@rq),0,1)))),
            max_joint_error_rad=float(joint_error.max()),worst_motor_id=int(mapping[joint_error.argmax()])))
    samples=run.get('samples',[])
    out=dict(status='provisional_new_start_image_comparison',source=str(folder),registration=reg,
        comparisons=comparisons,placement_camera_ids=run.get('placement_snapshot',{}).get('detected_ids'),
        min_voltage_v=min(min(r['voltage_v']) for r in samples),
        peak_current_ma=max(abs(v) for r in samples for v in r['current_ma']),
        all_torque_off_verified=run['all_torque_off_verified'],settings_restored=run['settings_restored'],
        measured_joint_angles_sim_order=measured.tolist(),
        limitations=['Single-marker pose may have planar ambiguity and depth bias.',
                    'ID40-to-ID41 rigidity after movement not independently verified.',
                    'Camera/marker-to-model transform remains provisional.'],
        hardware_policy_authorized=False,motor_commands_sent=0)
    (ROOT/'sac_revised_placement_audit_result.json').write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps({k:v for k,v in out.items() if k not in ('registration','measured_joint_angles_sim_order')},indent=2))

if __name__=='__main__':main()
