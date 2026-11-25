"""Optional CAD fingertip contact model for OFFLINE comparison only.

Replaces four box contact approximations with their existing visual CAD hulls.
Preserves joint ranges, masses/inertias and contact settings. Does not install
this geometry into an execution gate or change robot settings.
"""
import copy
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from book_support_model_20261024 import build_book

REVISION='CAD_tip_contact_diagnostic_v1_20261025'


def build(env,scene_path,*,book_projection_mm=-10.,middle_ring_hulls=True,use_cad_tips=True):
    assets=copy.deepcopy(dict(env._model_assets));replaced=[];proxies=[];middle_thumb_proxies=[]
    for name,blob in list(assets.items()):
        if not str(name).endswith('.xml'):continue
        xml=ET.fromstring(blob)
        for body in xml.iter('body'):
            bn=body.get('name','')
            if use_cad_tips and bn in ('if_ds','mf_ds','rf_ds','th_ds'):
                finger=bn[:2];tip=next(g for g in body.findall('geom') if g.get('name')==finger+'_tip')
                visuals=[g for g in body.findall('geom') if g.get('mesh') in ('tip','thumb_tip')]
                if len(visuals)!=1:raise ValueError('Expected unique fingertip CAD visual')
                visual=visuals[0]
                # Clone the visual INCLUDING its default class. Retaining the
                # old box's default class would apply its offset a second time.
                replacement=copy.deepcopy(visual)
                replacement.attrib.update(name=finger+'_tip',type='mesh',group='3',density='0',mass='0')
                native=env.mj_model.geom(finger+'_tip').id
                for attr in ('friction','solref','solimp'):
                    replacement.set(attr,' '.join(map(str,getattr(env.mj_model,'geom_'+attr)[native])))
                for attr in ('contype','conaffinity','condim','priority'):
                    replacement.set(attr,str(int(getattr(env.mj_model,'geom_'+attr)[native])))
                for attr in ('margin','gap','solmix'):
                    replacement.set(attr,str(float(getattr(env.mj_model,'geom_'+attr)[native])))
                index=list(body).index(tip);body.remove(tip);body.insert(index,replacement)
                replaced.append(finger+'_tip')
            mf,rf=bn.startswith('mf_'),bn.startswith('rf_')
            thumb=bn in ('th_px','th_ds')
            original_visuals=[g for g in body.findall('geom') if g.get('mesh') is not None
                and not g.get('name','').endswith('_tip') and g.get('group')!='4']
            if mf or thumb:
                for i,geom in enumerate(original_visuals):
                    g=copy.deepcopy(geom);g.attrib.pop('class',None)
                    gn=f'cadtip_mf_th_{bn}_{i}'
                    g.attrib.update(name=gn,type='mesh',density='0',mass='0',group='4',rgba='0 0 0 0',
                        contype='16' if mf else '32',conaffinity='32' if mf else '16')
                    body.append(g);middle_thumb_proxies.append(gn)
            if middle_ring_hulls and (mf or rf):
                # Existing collision geoms now also have a mesh attribute;
                # select VISUAL geoms only, never copy the replacement twice.
                for i,geom in enumerate(original_visuals):
                    g=copy.deepcopy(geom);g.attrib.pop('class',None)
                    gn=f'cadtip_mf_rf_{bn}_{i}'
                    g.attrib.update(name=gn,type='mesh',density='0',mass='0',group='4',rgba='0 0 0 0',
                        contype='64' if mf else '128',conaffinity='128' if mf else '64')
                    body.append(g);proxies.append(gn)
        assets[name]=ET.tostring(xml,encoding='utf-8')
    if use_cad_tips and sorted(replaced)!=['if_tip','mf_tip','rf_tip','th_tip']:raise ValueError('Expected four replacements')
    wrapped=SimpleNamespace(mj_model=env.mj_model,xml_path=env.xml_path,_model_assets=assets)
    model,manifest=build_book(wrapped,Path(scene_path),book_projection_mm=book_projection_mm,
        book_ylo_mm=-500.,book_yhi_mm=500.,support_projection_mm=50.,additional_self_hulls=False)
    hand_mask=np.arange(model.nbody)!=model.body('cube').id
    for a,b in ((model.jnt_range,env.mj_model.jnt_range),
                (model.actuator_ctrlrange,env.mj_model.actuator_ctrlrange),
                (model.body_mass[hand_mask],env.mj_model.body_mass[hand_mask]),
                (model.body_inertia[hand_mask],env.mj_model.body_inertia[hand_mask])):
        if not np.array_equal(a,b):raise RuntimeError('CAD contact changed native hand properties')
    for tip in replaced:
        g=model.geom(tip).id
        if model.geom_type[g]!=mujoco.mjtGeom.mjGEOM_MESH:raise RuntimeError('Tip replacement is not a mesh')
        visual=[j for j in range(model.ngeom) if model.geom_bodyid[j]==model.geom_bodyid[g]
            and model.geom_group[j]==2 and model.geom_type[j]==mujoco.mjtGeom.mjGEOM_MESH
            and model.geom_dataid[j]==model.geom_dataid[g]]
        if len(visual)!=1:raise RuntimeError('Expected unique source CAD visual')
        j=visual[0]
        if not np.allclose(model.geom_pos[g],model.geom_pos[j],atol=1e-12) or abs(abs(np.dot(model.geom_quat[g],model.geom_quat[j]))-1)>1e-12:
            raise RuntimeError('CAD collision and source visual transforms differ')
        native=env.mj_model.geom(tip).id
        for attr in ('geom_friction','geom_solref','geom_solimp','geom_contype','geom_conaffinity','geom_condim','geom_priority'):
            if not np.array_equal(getattr(model,attr)[g],getattr(env.mj_model,attr)[native]):
                raise RuntimeError('Tip contact parameter changed: '+attr)
    if middle_ring_hulls and (len(proxies)!=10 or any(model.geom_type[model.geom(n).id]!=mujoco.mjtGeom.mjGEOM_MESH for n in proxies)):
        raise RuntimeError('Middle/ring proxies must be exactly ten CAD mesh clones')
    manifest.update(fingertip_collision_model=REVISION if use_cad_tips else 'native_box_same_mesh_proxy_coverage_v1',replaced_tip_names=replaced,
        CAD_collision_and_visual_compiled_transforms_equal=True,
        middle_thumb_proxies=middle_thumb_proxies,middle_ring_proxies=proxies,
        physical_tip_geometry_calibration_verified=False,comparison_only=True,
        middle_ring_mesh_model_revision='explicit_mesh_type_v1_20261025')
    return model,manifest
