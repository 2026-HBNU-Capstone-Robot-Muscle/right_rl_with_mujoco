"""Compare the exact recorded RF posture with clipping, using geometry only.

The recorded value is kept intact. No device limits or original models change.
Assumed cube poses are saved model poses, not a reconstruction of the user's cube.
"""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,ROOT,assets,Guard,save_json,sha,TICK
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from convex_distance_check_20261104 import CheckedDistance


def main():
    folder=OUT/'next_cycle/handoff_refinement_20261110/recorded_ring_reassessment'
    if folder.exists():raise RuntimeError('Preserve previous reassessment')
    folder.mkdir()
    source=ROOT/'ring_reference_readonly_20261109_124719_945552/result.json'
    measured=json.loads(source.read_text());a={r['id']:r for r in measured['snapshots'][0]['motors']}
    b={r['id']:r for r in measured['snapshots'][1]['motors']}
    sim_ids=np.array(SIM_TO_MOTOR_IDS);rf=np.arange(8,12)
    target=np.array([(a[int(mid)]['position_ticks']-2048)*TICK for mid in sim_ids[rf]])
    assert all(a[int(mid)]['position_ticks']==b[int(mid)]['position_ticks'] for mid in sim_ids[rf])
    m,_,_,_,provenance=assets('active_ccd50');guard=Guard(m);cube=m.geom('cube').id
    rf_geoms=[g for g in range(m.ngeom) if m.geom(g).name.startswith('rf_') and int(m.geom_type[g]) in (6,7)]
    distances=CheckedDistance(m,cube,rf_geoms)
    jid=int(m.actuator_trnid[9,0])
    limits_source=ROOT/'right_hand_rotation_20261023/native_motor_limits.json'
    export_source=ROOT/'export_native_motor_limits_20261023.py'
    limit_info=dict(motor_id=14,joint_name=m.joint(jid).name,
        exported_limit_json=str(limits_source.relative_to(ROOT)),exported_limit_sha256=sha(limits_source),
        exporter=str(export_source.relative_to(ROOT)),exporter_sha256=sha(export_source),
        source_is_intersection_of_model_joint_and_actuator_ranges=True,
        hardware_EEPROM_limit_was_not_read_to_derive_this_bound=True,
        model_joint_range_deg=np.degrees(m.jnt_range[jid]).tolist(),
        model_actuator_ctrlrange_deg=np.degrees(m.actuator_ctrlrange[9]).tolist(),
        software_upper_ticks=float(guard.hi[13]),recorded_ticks=int(a[14]['position_ticks']),
        recorded_goal_ticks=int(a[14]['goal_ticks_readonly']),
        record_above_software_upper_deg=float((a[14]['position_ticks']-guard.hi[13])*TICK*180/np.pi))
    sources=[
        ('middle_probe_grasp',ROOT/'right_hand_rotation_20261109/middle_probe_reconstruction_linux/checkpoint_927.npz'),
        ('middle_probe_clearance',ROOT/'right_hand_rotation_20261109/middle_probe_reconstruction_linux/checkpoint_1042.npz'),
        ('historical_first_rotation_hold',OUT/'next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_schema_v2_confirmation/end_state.npz')]
    results=[]
    for name,path in sources:
        qpos=np.load(path,allow_pickle=False)['qpos'].copy()
        for mode in ('source','clipped_reference','exact_recorded_reference'):
            d=mujoco.MjData(m);d.qpos[:]=qpos
            if mode!='source':d.qpos[rf]=np.clip(target,guard.qlo[rf],guard.qhi[rf]) if mode=='clipped_reference' else target
            d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d)
            g=Guard(m);g.check(d)
            gaps=[dict(geom=m.geom(gid).name,signed_SAT_gap_mm=distances.gap(d,gid)*1000) for gid in rf_geoms]
            contacts=[]
            for c in d.contact:
                x,y=int(c.geom1),int(c.geom2)
                if cube not in (x,y):continue
                other=y if x==cube else x
                if m.geom(other).name.startswith('rf_'):
                    contacts.append(dict(geom=m.geom(other).name,distance_mm=float(c.dist*1000)))
            rows=dict(source=name,source_path=str(path.relative_to(ROOT)),source_sha256=sha(path),mode=mode,
                ring_ids_sim_order=sim_ids[rf].tolist(),ring_pose_ticks=(d.qpos[rf]/TICK+2048).tolist(),
                closest_rf=sorted(gaps,key=lambda r:r['signed_SAT_gap_mm'])[:4],rf_cube_contacts=contacts,
                original_guard=g.last,cube_center=d.qpos[-7:-4].tolist(),
                only_static_geometry_query=True,not_a_motion_or_holding_success=True)
            results.append(rows)
            np.savez_compressed(folder/(name+'_'+mode+'.npz'),qpos=d.qpos,ctrl=d.ctrl)
    r=dict(status='exact_recorded_RF_vs_clipped_static_geometry',limit_provenance=limit_info,
        recorded_source_sha256=sha(source),model_provenance=provenance,results=results,
        original_model_and_software_limits_unchanged=True,
        exact_reference_is_not_discarded_for_exceeding_model_bound=True,
        no_physical_cube_pose_was_measured_for_this_record=True,
        original_limits_rejection_is_not_a_mechanical_unreachability_proof=True,
        devices_opened=0,hardware_execution_allowed=False)
    save_json(folder/'geometry_audit.json',r)
    (folder/'source_audit_recorded_ring_pose.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps(dict(limit_provenance=limit_info,comparisons=[dict(source=x['source'],mode=x['mode'],
        rf_gap_mm=x['closest_rf'][0]['signed_SAT_gap_mm'],self_mm=x['original_guard']['self_mm'],
        cube_mm=x['original_guard']['cube_mm'],range_rad=x['original_guard']['range_rad']) for x in results])),flush=True)


if __name__=='__main__':main()
