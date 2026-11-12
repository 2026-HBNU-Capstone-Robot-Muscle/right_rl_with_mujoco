"""Offline FK and housing diagnostics for two saved matching-pose side views."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import copy
import json
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from scipy.spatial import ConvexHull
from mujoco_playground import registry
from book_support_model_20261024 import build_book
from hand_motor_mapping import ticks_to_sim, SIM_TO_MOTOR_IDS

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/middle_ring_side_review'


def main():
    source=json.loads((OUT/'paired_capture_summary.json').read_bytes())
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg)
    assets=copy.deepcopy(dict(env._model_assets));hulls=[]
    for name,blob in list(assets.items()):
        if not str(name).endswith('.xml'):continue
        xml=ET.fromstring(blob)
        for body in xml.iter('body'):
            bname=body.get('name','');mf=bname.startswith('mf_');rf=bname.startswith('rf_')
            if not (mf or rf):continue
            for i,geom in enumerate(list(body.findall('geom'))):
                if geom.get('mesh') is None:continue
                g=copy.deepcopy(geom);g.attrib.pop('class',None)
                gn=f'paired_side_mf_rf_{bname}_{i}'
                g.attrib.update(name=gn,type='mesh',density='0',mass='0',group='4',rgba='0 0 0 0',
                    contype='64' if mf else '128',conaffinity='128' if mf else '64')
                body.append(g);hulls.append(gn)
        assets[name]=ET.tostring(xml,encoding='utf-8')
    wrapped=SimpleNamespace(mj_model=env.mj_model,xml_path=env.xml_path,_model_assets=assets)
    m,manifest=build_book(wrapped,OUT/'paired_side_scene.xml',book_projection_mm=-10.,
        book_ylo_mm=-500.,book_yhi_mm=500.,support_projection_mm=50.)
    if len(hulls)!=10 or any(m.geom_type[m.geom(n).id]!=mujoco.mjtGeom.mjGEOM_MESH for n in hulls):
        raise RuntimeError('Middle/ring housing proxy must be a mesh')
    manifest['middle_ring_mesh_model_revision']='explicit_mesh_type_v1_20261025'
    output=[];d=mujoco.MjData(m)
    for capture in source['captures']:
        raw=np.array(capture['raw_ticks']);q=ticks_to_sim(raw)
        d.qpos[:16]=q;d.qpos[-7:]=[0,0,2,1,0,0,0];mujoco.mj_forward(m,d)
        palm=m.body('palm').id;hand={palm}
        for b in range(m.nbody):
            if int(m.body_parentid[b]) in hand:hand.add(b)
        meshes=[]
        for g in range(m.ngeom):
            if int(m.geom_bodyid[g]) not in hand or m.geom_group[g]>=3:continue
            if m.geom_type[g]!=mujoco.mjtGeom.mjGEOM_MESH:continue
            idx=int(m.geom_dataid[g]);start=int(m.mesh_vertadr[idx]);count=int(m.mesh_vertnum[idx])
            vertices=m.mesh_vert[start:start+count].copy()
            try:vertices=vertices[ConvexHull(vertices).vertices]
            except Exception:pass
            world=vertices@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
            meshes.append(dict(geom=m.geom(g).name,body=m.body(m.geom_bodyid[g]).name,
                vertices_world_m=world.tolist()))
        contacts=[]
        for c in d.contact:
            if c.dist>=-1e-7:continue
            a,b=int(c.geom1),int(c.geom2);na,nb=m.geom(a).name,m.geom(b).name
            if int(m.geom_bodyid[a]) not in hand and int(m.geom_bodyid[b]) not in hand:continue
            kind='middle_ring_mesh_hulls' if na in hulls and nb in hulls else 'native_or_existing_support_and_self'
            contacts.append(dict(pair=sorted((na,nb)),model_overlap_mm=-float(c.dist)*1000,
                position_world_m=c.pos.tolist(),kind=kind,physical_contact_verified=False))
        mf_hulls=[m.geom(n).id for n in hulls if m.body(int(m.geom_bodyid[m.geom(n).id])).name.startswith('mf_')]
        rf_hulls=[m.geom(n).id for n in hulls if m.body(int(m.geom_bodyid[m.geom(n).id])).name.startswith('rf_')]
        assert mf_hulls and rf_hulls and not (set(mf_hulls)&set(rf_hulls))
        gaps=[]
        for a in mf_hulls:
            for b in rf_hulls:
                witness=np.zeros(6)
                distance=mujoco.mj_geomDistance(m,d,a,b,.2,witness)
                gaps.append(dict(pair=[m.geom(a).name,m.geom(b).name],model_gap_mm=float(distance)*1000,
                    witness_world_m=witness.reshape(2,3).tolist()))
        gaps.sort(key=lambda item:item['model_gap_mm'])
        outside=[]
        for i,j in enumerate(m.actuator_trnid[:,0]):
            lo,hi=m.jnt_range[j]
            if q[i]<lo or q[i]>hi:
                outside.append(dict(motor_id=int(SIM_TO_MOTOR_IDS[i]),angle_rad=float(q[i]),
                    range_rad=[float(lo),float(hi)],distance_outside_rad=float(max(lo-q[i],q[i]-hi))))
        row=dict(folder=capture['folder'],result_sha256=capture['result_sha256'],
            photo_sha256=capture['image_sha256'],raw_ticks=raw.tolist(),meshes=meshes,
            joints=[dict(name=m.joint(int(j)).name,motor_id=int(SIM_TO_MOTOR_IDS[i]),world_m=d.xanchor[j].tolist())
                for i,j in enumerate(m.actuator_trnid[:,0])],
            palm_position_world_m=d.xpos[palm].tolist(),palm_rotation_world=d.xmat[palm].reshape(3,3).tolist(),
            middle_base_position_world_m=d.xpos[m.body('mf_bs').id].tolist(),
            contacts=contacts,closest_middle_ring_hull_pairs=gaps[:5],native_limit_outside=outside,joint_angles_clipped_or_offset_fitted=False,
            housing_hulls_are_convex_diagnostics_not_physical_depth_measurements=True)
        output.append(row)
        print('POSE',capture['folder'],'outside',outside,'MR_hull_contacts',
            [c for c in contacts if c['kind']=='middle_ring_mesh_hulls'],flush=True)
    (OUT/'paired_native_geometry.json').write_text(json.dumps(dict(captures=output,
        support=manifest,diagnostic_middle_ring_hulls=hulls,geometry_calibration_verified=False,
        hardware_execution_allowed=False,hardware_commands_sent=0)),encoding='utf-8')
    from PIL import Image,ImageDraw
    renderer=mujoco.Renderer(m,height=480,width=640);option=mujoco.MjvOption();option.geomgroup[4]=0
    colors={'if_':[.15,.45,1,1],'mf_':[.12,.8,.2,1],'rf_':[1,.7,.1,1],'th_':[.9,.15,.12,1]}
    for g in range(m.ngeom):
        name=m.body(int(m.geom_bodyid[g])).name
        if m.geom_group[g]>=3:continue
        for prefix,color in colors.items():
            if name.startswith(prefix):m.geom_rgba[g]=color;m.geom_matid[g]=-1
        if name=='cube' or m.geom(g).name.startswith('support_'):m.geom_group[g]=4
    sheet=Image.new('RGB',(1280,960),(20,20,20))
    for i,(azimuth,elevation) in enumerate(((130,-30),(45,-30),(220,-45),(130,-75))):
        camera=mujoco.MjvCamera();camera.lookat[:]=[.12,0,-.005];camera.distance=.35
        camera.azimuth=azimuth;camera.elevation=elevation
        renderer.update_scene(d,camera=camera,scene_option=option)
        im=Image.fromarray(renderer.render());draw=ImageDraw.Draw(im)
        draw.rectangle((0,0,640,38),fill=(20,20,20))
        draw.text((10,8),'Saved OFF encoders. Green=middle Gold=ring Red=thumb',fill='white')
        draw.text((10,23),'Native geometry; no angle clipping or fitted joint zeros',fill='white')
        sheet.paste(im,((i%2)*640,(i//2)*480))
    sheet.save(OUT/'paired_pose_model_views.jpg');renderer.close()


if __name__=='__main__':main()
