"""Conservative middle/thumb mesh-hull sensitivity model; offline use only.

Proxies interact with each other only, never with the cube or support. Native
hand contacts remain enabled. Convex mesh hulls can overestimate concave shells.
"""
import copy
from types import SimpleNamespace
import xml.etree.ElementTree as ET
import numpy as np
from sac_wood_glass_model import build

def build_with_middle_thumb_hulls(env):
    assets=copy.deepcopy(dict(env._model_assets));added=[]
    for key,value in list(assets.items()):
        if not str(key).endswith('.xml'):continue
        tree=ET.fromstring(value);changed=False
        for body in tree.iter('body'):
            name=body.get('name','')
            middle=name.startswith('mf_');thumb=name in ('th_px','th_ds')
            if not (middle or thumb):continue
            for index,geom in enumerate(list(body.findall('geom'))):
                if geom.get('mesh') is None:continue
                # One copy of each visible mesh; do not copy native collision boxes.
                proxy=copy.deepcopy(geom);proxy.attrib.pop('class',None)
                proxy_name=f'{name}_offline_hull_{index}'
                attrs=dict(name=proxy_name,type='mesh',density='0',mass='0',group='4',
                           rgba='0 0 0 0',contype='16' if middle else '32',conaffinity='32' if middle else '16')
                for attr,val in attrs.items():proxy.set(attr,val)
                body.append(proxy);added.append(dict(name=proxy_name,body=name,source_mesh=geom.get('mesh')));changed=True
        if changed:assets[key]=ET.tostring(tree,encoding='utf-8')
    assert any(x['body'].startswith('mf_') for x in added) and any(x['body'].startswith('th_') for x in added)
    wrapped=SimpleNamespace(mj_model=env.mj_model,xml_path=env.xml_path,_model_assets=assets)
    model,manifest=build(wrapped)
    assert model.nq==env.mj_model.nq and model.nu==env.mj_model.nu
    assert np.allclose(model.body_mass[:19],env.mj_model.body_mass[:19])
    assert np.allclose(model.body_inertia[:19],env.mj_model.body_inertia[:19])
    manifest['additional_middle_thumb_hull_proxies']=added
    manifest['additional_proxy_caveat']='Offline sensitivity: convex visual-mesh hulls overapproximate concavities; no cube contacts, no fitted offsets.'
    return model,manifest
