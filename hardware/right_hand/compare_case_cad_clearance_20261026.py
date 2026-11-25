"""Compare primitive housing collisions with original CAD hull distances.

Adds collision-ready diagnostic copies but changes no native collision geoms.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import copy,json,xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261026/case_cad_comparison'
def main():
    OUT.mkdir(exist_ok=True);cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);assets=copy.deepcopy(dict(env._model_assets));proxies={};sources=[]
    for name,blob in list(assets.items()):
        if not str(name).endswith('.xml'):continue
        xml=ET.fromstring(blob)
        for body in xml.iter('body'):
            bn=body.get('name','')
            if bn not in ('if_ds','mf_ds','th_px','th_ds','palm'):continue
            visuals=[g for g in body.findall('geom') if g.get('mesh') is not None and g.get('mesh') not in ('tip','thumb_tip') and not g.get('name','').endswith('_tip')]
            for i,g in enumerate(visuals):
                clone=copy.deepcopy(g);gn=f'case_cad_diag_{bn}_{i}';clone.attrib.update(name=gn,type='mesh',group='4',contype='1024',conaffinity='0',density='0',mass='0');body.append(clone);proxies.setdefault(bn,[]).append(gn);sources.append(dict(name=gn,body=bn,mesh=g.get('mesh'),source_attributes=dict(g.attrib)))
        assets[name]=ET.tostring(xml,encoding='utf-8')
    wrapped=SimpleNamespace(mj_model=env.mj_model,xml_path=env.xml_path,_model_assets=assets);m,manifest=build(wrapped,OUT/'scene.xml');d=mujoco.MjData(m)
    for item in sources:
        g=m.geom(item['name']).id;matches=[j for j in range(m.ngeom) if m.geom_bodyid[j]==m.geom_bodyid[g] and m.geom_group[j]==2 and m.geom_dataid[j]==m.geom_dataid[g] and m.geom_type[j]==mujoco.mjtGeom.mjGEOM_MESH]
        if len(matches)!=1:raise RuntimeError('Cannot identify unique visual CAD source')
        j=matches[0]
        if not np.allclose(m.geom_pos[g],m.geom_pos[j],atol=1e-12) or abs(abs(np.dot(m.geom_quat[g],m.geom_quat[j]))-1)>1e-12:raise RuntimeError('Diagnostic CAD transform differs from its visual source')
    graph={n:int(m.mesh_graphadr[int(m.geom_dataid[m.geom(n).id])]) for ns in proxies.values() for n in ns}
    if any(v<0 for v in graph.values()):raise RuntimeError('A diagnostic mesh has no compiled convex graph; do not query it')
    cube=m.geom('cube').id;raw=json.loads((ROOT/'sequence_flip_poses.json').read_bytes());actual=json.loads((ROOT/'loaded_repeat_20261026_170932_163649/result.json').read_bytes())['rotation_position_ticks'];rows=[]
    poses=[('original_P0',np.asarray(raw['poses'][0]['raw_ticks']),None),('actual_after_flip',np.asarray(actual),None)]
    end=json.loads((ROOT/'right_hand_rotation_20261026/saved_marker_registration/saved_after_flip_outline_calibrated_hypothesis.json').read_bytes())
    for r in end['source_frames']:
        quat=Rotation.from_matrix(r['R_cube_world']).as_quat();poses.append((r['image'],np.asarray(actual),np.r_[r['center_world_m'],quat[[3,0,1,2]]]))
    for label,goal,state in poses:
        d.qpos[:16]=ticks_to_sim(goal);d.qpos[-7:]=np.r_[[0,0,10],[1,0,0,0]] if state is None else state;d.qvel[:]=0;mujoco.mj_forward(m,d);native=[]
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2)
            if c.dist<-.0001:native.append(dict(pair=[m.geom(a).name,m.geom(b).name],depth_mm=-float(c.dist)*1000))
        case_pair=[]
        for a in proxies['if_ds']:
            for b in proxies['mf_ds']:case_pair.append(dict(pair=[a,b],distance_mm=float(mujoco.mj_geomDistance(m,d,m.geom(a).id,m.geom(b).id,1.,None)*1000)))
        cubes=[]
        if state is not None:
            for body in ('th_px','th_ds','palm'):
                for name in proxies.get(body,[]):cubes.append(dict(geom=name,body=body,distance_mm=float(mujoco.mj_geomDistance(m,d,cube,m.geom(name).id,1.,None)*1000)))
        row=dict(pose=label,native_contacts=native,index_middle_case_cad_distances=case_pair,cube_case_cad_distances=cubes);rows.append(row);print(json.dumps(row),flush=True)
    result=dict(status='native_box_vs_original_visual_cad_hull_diagnostic',source_meshes=sources,compiled_mesh_graph_addresses=graph,cases=rows,
        palm_convex_hull_may_fill_real_concavities=True,original_collision_geometries_preserved=True,native_hand_properties_preserved=True,not_a_physical_geometry_calibration=True,hardware_execution_allowed=False,real_devices_opened=False)
    (OUT/'review.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
