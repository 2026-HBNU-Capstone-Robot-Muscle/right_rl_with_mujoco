"""Native MuJoCo geometry of directly paired photos/encoders. No devices.

Unchanged marker/model candidate, model ranges and physical mapping. All pose
branches are retained. Box separation is independently checked by OBB geometry.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from mujoco_playground import registry

from box_clearance import clearance,SIGNS
from hand_motor_mapping import ticks_to_sim
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from sac_cube_model import set_cube_physics

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'cube_pose_readonly_20261023_171233_601943'


def main():
    blob=(FOLDER/'direct_marker_pose_review.json').read_bytes();source=json.loads(blob)
    assert source['directly_paired_frames']==10 and not source['old_joint_values_used']
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build_with_middle_thumb_hulls(env)
    set_cube_physics(m,side_m=.055,mass_kg=.1)
    d=mujoco.MjData(m);cube=m.geom('cube').id;tips=('if_tip','mf_tip','rf_tip','th_tip')
    rows=[];representatives=[]
    for hyp in source['hypotheses']:
        q=ticks_to_sim(hyp['measured_motor_ticks']);d.qpos[:16]=q
        R=np.array(hyp['cube_rotation_world']);center=np.array(hyp['cube_center_world_m'])
        d.qpos[-7:]=np.r_[center,Rotation.from_matrix(R).as_quat()[[3,0,1,2]]]
        d.qvel[:]=0;mujoco.mj_forward(m,d)
        distances={};geometry=[]
        for name in tips:
            g=m.geom(name).id
            assert m.geom_type[g]==mujoco.mjtGeom.mjGEOM_BOX
            gR=d.geom_xmat[g].reshape(3,3)
            gap,overlap=clearance(center,R,np.full(3,.0275),d.geom_xpos[g],gR,m.geom_size[g])
            distances[name]=dict(independent_box_gap_mm=gap*1000,boxes_overlap=overlap,
                mujoco_raw_distance_mm=float(mujoco.mj_geomDistance(m,d,cube,g,1,None))*1000)
            geometry.append(dict(name=name,center_world_m=d.geom_xpos[g].tolist(),
                box_vertices_world_m=((SIGNS*m.geom_size[g])@gR.T+d.geom_xpos[g]).tolist()))
        contacts=[]
        for c in d.contact:
            if cube in (int(c.geom1),int(c.geom2)):
                other=int(c.geom2) if int(c.geom1)==cube else int(c.geom1)
                contacts.append(dict(other=m.geom(other).name,signed_distance_mm=float(c.dist)*1000))
        limits=[]
        for a in range(16):
            joint=int(m.actuator_trnid[a,0]);low,high=m.jnt_range[joint]
            outside=max(float(low-q[a]),float(q[a]-high),0.)
            if outside:limits.append(dict(joint=m.joint(joint).name,angle=float(q[a]),
                native_range=[float(low),float(high)],excess_rad=outside))
        rows.append(dict(image=hyp['image'],source=hyp['source'],palm_branch=hyp['palm_branch'],
            cube_branch=hyp['cube_branch'],tip_distances=distances,cube_contacts=contacts,
            native_limit_excess=limits,maximum_cube_overlap_mm=max((max(0.,-c['signed_distance_mm']) for c in contacts),default=0.)))
        if hyp['image']=='pair_05.png':
            representatives.append(dict(**hyp,tip_boxes=geometry,
                joint_anchors_world_m={m.joint(i).name:d.xanchor[i].tolist() for i in range(16)},
                cube_vertices_world_m=((SIGNS*.0275)@R.T+center).tolist()))
    summaries=[]
    keys=sorted({(r['source'],r['palm_branch'],r['cube_branch']) for r in rows},key=str)
    for key in keys:
        chosen=[r for r in rows if (r['source'],r['palm_branch'],r['cube_branch'])==key]
        summary=dict(source=key[0],palm_branch=key[1],cube_branch=key[2],frames=len(chosen),
            maximum_cube_overlap_mm=max(r['maximum_cube_overlap_mm'] for r in chosen),tips={})
        for tip in tips:
            gaps=[r['tip_distances'][tip]['independent_box_gap_mm'] for r in chosen]
            summary['tips'][tip]=dict(min_gap_mm=min(gaps),max_gap_mm=max(gaps),
                overlap_frames=sum(r['tip_distances'][tip]['boxes_overlap'] for r in chosen))
        summaries.append(summary)
    result=dict(status='direct_paired_native_geometry_diagnostic',source_sha256=hashlib.sha256(blob).hexdigest(),
        native_limit_ranges_unchanged=True,joint_offsets_fitted=False,old_joint_values_used=False,
        rows=rows,representatives=representatives,branch_summaries=summaries,model_manifest=manifest,
        marker_to_model_registration_verified=False,hardware_execution_allowed=False,
        servo_writes_sent=0,real_devices_opened=False,
        limitations=['Box clearances and MuJoCo contacts are model diagnostics, not measured physical forces.',
            'The user-estimated marker mount is unchanged; it can bias all projected contacts.',
            'The grid diagnostic assumes a solved green face with equal tile pitch.',
            'Static OFF pose comparison does not establish a safe or successful motor path.'])
    (FOLDER/'native_geometry_review.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(summaries,indent=2),flush=True)
    print('Native limit excess',rows[0]['native_limit_excess'],flush=True)


if __name__=='__main__':main()
