"""Native offline obstacle model from the user's measured wood/glass stack.

Fixed motor-mount front is the CAD palm's outermost +X plane, 3 mm.
Bottom is the large +Z plane, 11.3 mm; palm-local +Z points underneath.
Unmeasured width/depth are conservative finite extents, not measurements.
"""
import json
import copy
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from sac_cube_model import set_cube_physics
ROOT=Path(__file__).resolve().parent

def build(env,projection_mm=None,glass_mm=None,front_reference_mm=3.,bottom_reference_mm=11.3,housing_colliders=True):
    measured=json.loads((ROOT/'wood_glass_support_measurements.json').read_text())
    projection=measured['wood_projection_from_fixed_motor_mount_front_mm'] if projection_mm is None else projection_mm
    thickness=measured['glass_thickness_mm'] if glass_mm is None else glass_mm
    if projection<0 or thickness<=0: raise ValueError('Invalid support dimensions')
    baseline=env.mj_model; data=mujoco.MjData(baseline); mujoco.mj_forward(baseline,data)
    palm=baseline.body('palm').id; rotation=data.xmat[palm].reshape(3,3); origin=data.xpos[palm]
    quat=np.empty(4); mujoco.mju_mat2Quat(quat,rotation.ravel())
    # Boxes are world-fixed separate geometries. Putting them on the palm body
    # would let parent-child collision filtering hide some finger contacts.
    tree=ET.fromstring(Path(env.xml_path).read_text()); world=tree.find('worldbody')
    if world is None:raise ValueError('Scene has no worldbody')
    front=front_reference_mm+projection; rear=-130.; ylo,yhi=-125.,65.
    bounds={
        'support_glass': [[rear,ylo,bottom_reference_mm],[front,yhi,bottom_reference_mm+thickness]],
        'support_wood': [[rear,ylo,bottom_reference_mm+thickness],[front,yhi,bottom_reference_mm+thickness+60.]],
    }
    for name,(lo,hi) in bounds.items():
        lo,hi=np.asarray(lo)/1000,np.asarray(hi)/1000
        center=origin+rotation@((lo+hi)/2)
        attrs=dict(name=name,type='box',size=' '.join(map(str,(hi-lo)/2)),
                   pos=' '.join(map(str,center)),quat=' '.join(map(str,quat)),
                   contype='4',conaffinity='11',friction='0.6 0.005 0.0001',
                   rgba='0.2 0.65 0.8 0.45' if name.endswith('glass') else '0.5 0.3 0.12 1',group='0')
        ET.SubElement(world,'geom',attrs)
    assets=dict(env._model_assets); added=[]
    if housing_colliders:
        for key,value in list(assets.items()):
            if not str(key).endswith('.xml'):continue
            asset=ET.fromstring(value); changed=False
            for body in asset.iter('body'):
                if body.get('name') not in ('if_bs','if_px','mf_bs','mf_px','rf_bs','rf_px'):continue
                for geom in list(body.findall('geom')):
                    if not geom.get('name','').endswith('_visual') or geom.get('mesh') is None:continue
                    proxy=copy.deepcopy(geom); proxy.attrib.pop('class',None)
                    proxy.set('name',geom.get('name')+'_support_proxy')
                    for attr,val in dict(type='mesh',contype='8',conaffinity='4',density='0',mass='0',group='4',rgba='0 0 0 0').items():proxy.set(attr,val)
                    body.append(proxy);added.append(proxy.get('name'));changed=True
            if changed:assets[key]=ET.tostring(asset,encoding='utf-8')
        if len(added)!=6:raise RuntimeError(f'Expected 6 external-only housing proxies, got {added}')
    model=mujoco.MjModel.from_xml_string(ET.tostring(tree,encoding='unicode'),assets=assets)
    model.opt.timestep=baseline.opt.timestep; model.opt.ccd_iterations=baseline.opt.ccd_iterations
    model.vis.global_.offwidth=1920; model.vis.global_.offheight=1440
    set_cube_physics(model,mass_kg=.100)
    audit=json.loads((ROOT/'sac_stage0_reset_audit_result.json').read_text())
    model.opt.gravity[:]=audit['gravity_m_s2']
    if model.nq!=baseline.nq or model.nu!=baseline.nu:raise RuntimeError('Support changed hand DOF')
    if not np.allclose(model.body_mass[:19],baseline.body_mass[:19]):raise RuntimeError('Housing proxy changed hand mass')
    manifest=dict(projection_mm=projection,glass_thickness_mm=thickness,
                  fixed_front_palm_x_mm=front_reference_mm,fixed_bottom_palm_z_mm=bottom_reference_mm,
                  bounds_palm_mm=bounds,wood_width_depth_thickness_measured=False,
                  external_only_housing_proxies=added,
                  caveat='Width 190mm, rear reach and wood depth are conservative analysis extents. Support friction is provisional. Existing hand collision primitives may omit housing details.')
    return model,manifest
