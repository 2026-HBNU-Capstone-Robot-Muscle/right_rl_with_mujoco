"""Identify joint reserve and sweep-certificate blockers at a saved waypoint."""
from pathlib import Path
import json
import numpy as np
from native_cycle import OUT, assets, restore, Guard, save_json, sha
from hand_motor_mapping import SIM_TO_MOTOR_IDS, ticks_to_sim
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';root=base/'robust_cycle_preparation_20261111'
    trial=root/'low_friction_loaded_waypoint_hold_v1';folder=root/'low_friction_waypoint_failure_diagnosis_v1'
    if folder.exists():raise RuntimeError('Preserve saved diagnosis')
    m,initial,_,_,provenance=assets('active_ccd50');m.geom_friction[:,0]=.25;g=Guard(m);cube=m.geom('cube').id
    parent=base/'robust_mu025_next_turn_teacher_v1'
    source=root/'low_friction_first_hold_state_v4/end_state.npz';start=dict(np.load(source,allow_pickle=False))
    start['qpos']=np.load(parent/'trajectory.npz',allow_pickle=False)['qpos'][368].copy()
    start['ctrl']=ticks_to_sim(np.load(parent/'research_goals.npz',allow_pickle=False)['goals'][367])
    end=dict(np.load(trial/'end_state.npz',allow_pickle=False));rows=[]
    for label,z in (('loaded_waypoint',start),('range_stop',end)):
        d=restore(m,z);over=np.maximum(g.qlo-d.qpos[:16],d.qpos[:16]-g.qhi);idx=int(np.argmax(over));cert=SweepCertificate(m,g.hand,.3)
        lower,bad=cert.query(d,d.geom_xpos[cube],d.geom_xmat[cube].reshape(3,3))
        rows.append(dict(stage=label,largest_signed_range_excess_rad=float(over[idx]),motor_id=int(SIM_TO_MOTOR_IDS[idx]),
            actual_joint_deg=float(np.degrees(d.qpos[idx])),lower_deg=float(np.degrees(g.qlo[idx])),upper_deg=float(np.degrees(g.qhi[idx])),
            commanded_joint_deg=float(np.degrees(d.ctrl[idx])),excess_ticks=float(max(0.,over[idx])*4096/(2*np.pi)),
            upper_separation_lower_bound_mm=lower,uncertified_geoms=[m.geom(int(cert.ids[i])).name for i in bad]))
    folder.mkdir();(folder/'source_audit_mu025_waypoint_failure.py').write_bytes(Path(__file__).read_bytes())
    r=dict(status='low_friction_waypoint_failure_identified',source_reports_sha256={str((trial/'review.json').relative_to(OUT)):sha(trial/'review.json')},rows=rows,
        upper_space_is_diagnostic_at_intermediate_waypoint_not_required_during_rotation=True,
        negative_certificate_bound_is_not_penetration_depth=True,devices_opened=0,hardware_execution_allowed=False)
    save_json(folder/'review.json',r);print(json.dumps(r),flush=True)


if __name__=='__main__':main()
