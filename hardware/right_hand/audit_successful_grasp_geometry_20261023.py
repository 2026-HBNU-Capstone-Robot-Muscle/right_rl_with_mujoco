"""MuJoCo comparison with same-run ON encoders. No fitted geometry or devices."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib
import json
from pathlib import Path
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics
from box_clearance import clearance,SIGNS

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'slow_grasp_demo_20261023_182105_573825'


def main():
    blob=(FOLDER/'on_paired_pose_review.json').read_bytes();source=json.loads(blob)
    assert source['directly_paired_ON_frames']>0 and source['old_OFF_angles_used'] is False
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build_with_middle_thumb_hulls(env);set_cube_physics(m,side_m=.055,mass_kg=.1)
    d=mujoco.MjData(m);cube=m.geom('cube').id;tips=('if_tip','mf_tip','rf_tip','th_tip')
    grid_images=sorted({h['image'] for h in source['hypotheses'] if h['source'].endswith('grid_diagnostic')})
    representative=grid_images[len(grid_images)//2] if grid_images else source['rows'][0]['image']
    rows=[];representatives=[]
    for h in source['hypotheses']:
        q=ticks_to_sim(h['measured_motor_ticks']);d.qpos[:16]=q
        R=np.array(h['cube_rotation_world']);center=np.array(h['cube_center_world_m'])
        d.qpos[-7:]=np.r_[center,Rotation.from_matrix(R).as_quat()[[3,0,1,2]]]
        d.qvel[:]=0;mujoco.mj_forward(m,d)
        distances={};boxes=[]
        for name in tips:
            g=m.geom(name).id;gR=d.geom_xmat[g].reshape(3,3)
            gap,overlap=clearance(center,R,np.full(3,.0275),d.geom_xpos[g],gR,m.geom_size[g])
            distances[name]=dict(independent_box_gap_mm=gap*1000,boxes_overlap=overlap,
                mujoco_distance_mm=float(mujoco.mj_geomDistance(m,d,cube,g,1,None))*1000)
            boxes.append(dict(name=name,vertices_world_m=((SIGNS*m.geom_size[g])@gR.T+d.geom_xpos[g]).tolist()))
        contacts=[]
        for c in d.contact:
            if cube in (int(c.geom1),int(c.geom2)):
                other=int(c.geom2) if int(c.geom1)==cube else int(c.geom1)
                contacts.append(dict(other=m.geom(other).name,signed_distance_mm=float(c.dist)*1000))
        rows.append(dict(image=h['image'],source=h['source'],palm_branch=h['palm_branch'],cube_branch=h['cube_branch'],
            tip_distances=distances,cube_contacts=contacts,
            maximum_cube_overlap_mm=max((max(0.,-c['signed_distance_mm']) for c in contacts),default=0.)))
        if h['image']==representative:
            representatives.append(dict(**h,tip_boxes=boxes,
                cube_vertices_world_m=((SIGNS*.0275)@R.T+center).tolist(),
                joint_anchors_world_m={m.joint(i).name:d.xanchor[i].tolist() for i in range(16)}))
    summaries=[]
    for key in sorted({(r['source'],r['palm_branch'],r['cube_branch']) for r in rows},key=str):
        chosen=[r for r in rows if (r['source'],r['palm_branch'],r['cube_branch'])==key]
        summary=dict(source=key[0],palm_branch=key[1],cube_branch=key[2],frames=len(chosen),tips={},
            maximum_cube_overlap_mm=max(r['maximum_cube_overlap_mm'] for r in chosen))
        for tip in tips:
            gaps=[r['tip_distances'][tip]['independent_box_gap_mm'] for r in chosen]
            summary['tips'][tip]=dict(median_gap_mm=float(np.median(gaps)),minimum_gap_mm=min(gaps),maximum_gap_mm=max(gaps),
                overlap_frames=sum(r['tip_distances'][tip]['boxes_overlap'] for r in chosen))
        summaries.append(summary)
    out=dict(status='paired_ON_native_geometry_diagnostic',source_sha256=hashlib.sha256(blob).hexdigest(),
        directly_paired_ON_frames=source['directly_paired_ON_frames'],rows=rows,representatives=representatives,
        branch_summaries=summaries,model_manifest=manifest,native_ranges_unchanged=True,
        joint_offsets_fitted=False,marker_to_model_registration_verified=False,old_OFF_angles_used=False,
        hardware_execution_allowed=False,hardware_commands_sent=0,
        limitations=['Distances refer to the unchanged provisional marker mount and collision boxes.',
            'They do not establish actual contact forces or a usable physical grip.',
            'Both planar pose branches are retained; no contact-driven pose selection.'])
    (FOLDER/'on_native_geometry_review.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
    print(json.dumps(summaries,indent=2),flush=True)


if __name__=='__main__':main()
