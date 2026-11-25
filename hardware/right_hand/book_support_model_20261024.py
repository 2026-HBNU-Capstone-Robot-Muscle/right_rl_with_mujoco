"""Measured book height with explicit, unmeasured footprint hypotheses.

Offline geometry for diagnostics and the separate EMPTY preparation screen.
Footprint cases remain hypotheses. Native joint/control ranges, masses and
mapping are preserved; this builder has no device access.
"""
import copy
import json
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from sac_wood_glass_model import build as build_glass_wood

ROOT=Path(__file__).resolve().parent
MEASUREMENTS=ROOT/'current_book_support_measurements_20261024.json'


def build_book(env,scene_path,*,book_projection_mm,book_yhi_mm,book_ylo_mm=-125.,
               support_projection_mm=50.,additional_self_hulls=True):
    measured=json.loads(MEASUREMENTS.read_text(encoding='utf-8'))
    height=float(measured['book_thickness_mm'])
    values=np.array([height,book_projection_mm,book_ylo_mm,book_yhi_mm,support_projection_mm],dtype=float)
    if not np.isfinite(values).all() or height<=0 or book_yhi_mm<=book_ylo_mm or book_projection_mm<=-133:
        raise ValueError('Invalid book-support hypothesis')
    baseline=env.mj_model;data=mujoco.MjData(baseline);mujoco.mj_forward(baseline,data)
    palm=baseline.body('palm').id;rotation=data.xmat[palm].reshape(3,3);origin=data.xpos[palm]
    quat=np.empty(4);mujoco.mju_mat2Quat(quat,rotation.ravel())
    lower=np.array([-130.,book_ylo_mm,11.3])/1000
    upper=np.array([3.+book_projection_mm,book_yhi_mm,11.3+height])/1000
    tree=ET.fromstring(Path(env.xml_path).read_text());world=tree.find('worldbody')
    if world is None:raise ValueError('No worldbody')
    ET.SubElement(world,'geom',dict(name='support_book',type='box',
        size=' '.join(map(str,(upper-lower)/2)),pos=' '.join(map(str,origin+rotation@((lower+upper)/2))),
        quat=' '.join(map(str,quat)),contype='4',conaffinity='11',
        friction='0.6 0.005 0.0001',rgba='0.9 0.9 0.8 1',group='0'))
    scene_path=Path(scene_path);scene_path.write_text(ET.tostring(tree,encoding='unicode'),encoding='utf-8')
    assets=copy.deepcopy(dict(env._model_assets));added=[]
    if additional_self_hulls:
        for key,value in list(assets.items()):
            if not str(key).endswith('.xml'):continue
            asset=ET.fromstring(value);changed=False
            for body in asset.iter('body'):
                name=body.get('name','');middle=name.startswith('mf_');thumb=name in ('th_px','th_ds')
                if not (middle or thumb):continue
                for index,geom in enumerate(list(body.findall('geom'))):
                    if geom.get('mesh') is None:continue
                    proxy=copy.deepcopy(geom);proxy.attrib.pop('class',None)
                    proxy_name=f'{name}_offline_hull_{index}'
                    for attr,val in dict(name=proxy_name,type='mesh',density='0',mass='0',group='4',
                            rgba='0 0 0 0',contype='16' if middle else '32',conaffinity='32' if middle else '16').items():
                        proxy.set(attr,val)
                    body.append(proxy);added.append(proxy_name);changed=True
            if changed:assets[key]=ET.tostring(asset,encoding='utf-8')
    wrapped=SimpleNamespace(mj_model=baseline,xml_path=scene_path,_model_assets=assets)
    model,manifest=build_glass_wood(wrapped,projection_mm=support_projection_mm,
                                  bottom_reference_mm=11.3+height)
    for a,b in ((model.jnt_range,baseline.jnt_range),(model.actuator_ctrlrange,baseline.actuator_ctrlrange),
                (model.body_mass[:19],baseline.body_mass[:19]),(model.body_inertia[:19],baseline.body_inertia[:19])):
        if not np.array_equal(a,b):raise RuntimeError('Book model changed native hand properties')
    manifest.update(fixed_body_bottom_palm_z_mm=11.3,book_thickness_mm_user_reported=height,
        book_bounds_palm_mm=[(lower*1000).tolist(),(upper*1000).tolist()],
        book_footprint_measured=False,additional_self_hulls=added,
        hardware_execution_allowed=False,geometry_calibration_verified=False)
    return model,manifest
