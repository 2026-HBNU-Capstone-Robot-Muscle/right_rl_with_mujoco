"""Check a free-route endpoint using native and independent convex geometry."""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,save_json,sha
from convex_pair_gap import ConvexPairGap
from directed_contact_state import tip_yaw_contact


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'reposition_then_second_rotation_20261111/standoff_endpoint_audit'
    if folder.exists():raise RuntimeError('Preserve endpoint audit')
    source=base/'robust_mu025_free_MF_standoff_route_v1/end_state.npz';target=base/'robust_mu025_directed_standoff_plan_v2/gap8mm/waypoint_reference_only.npz'
    m,_,_,_,_=assets('active_ccd50');m.geom_friction[:,0]=.25;cube=m.geom('cube').id;tip=m.geom('mf_tip').id;independent=ConvexPairGap(m,[cube,tip]);rows=[]
    expected=dict(np.load(target,allow_pickle=False))
    for label,path in (('reference_only',target),('actual_replayed_end',source)):
        z=dict(np.load(path,allow_pickle=False));d=restore(m,z);points=np.zeros(6);gap=float(mujoco.mj_geomDistance(m,d,cube,tip,.2,points))*1000
        rows.append(dict(stage=label,source=path.relative_to(OUT).as_posix(),source_sha256=sha(path),native_gap_mm=gap,closest_point_distance_mm=float(np.linalg.norm(points[3:]-points[:3])*1000),closest_points=points.tolist(),independent_convex_axis_gap_mm=independent.gap(d,cube,tip)*1000,
            MF_actual_difference_from_reference_deg=np.degrees(d.qpos[4:8]-expected['qpos'][4:8]).tolist(),MF_command_difference_from_reference_deg=np.degrees(d.ctrl[4:8]-expected['ctrl'][4:8]).tolist(),cube_center_m=d.geom_xpos[cube].tolist(),tip_center_m=d.geom_xpos[tip].tolist(),post_restore_contact=tip_yaw_contact(m,d,cube,tip,z['up'])))
    folder.mkdir();r=dict(status='standoff_endpoint_geometry_audit',rows=rows,devices_opened=0,hardware_execution_allowed=False);save_json(folder/'review.json',r);print(json.dumps(r),flush=True)


if __name__=='__main__':main()
