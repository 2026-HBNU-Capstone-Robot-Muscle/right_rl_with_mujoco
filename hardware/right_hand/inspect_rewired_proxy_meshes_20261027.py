"""Identify the exact colliders and compare their poses with source visuals."""
import importlib.util,json,os,xml.etree.ElementTree as ET
import numpy as np
os.environ.setdefault('MUJOCO_GL','egl')
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
import horizontal_quarter_plan_20261027 as p

def main():
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    bodies=[]
    for name,blob in env._model_assets.items():
        if not str(name).endswith('.xml'):continue
        xml=ET.fromstring(blob)
        for body in xml.iter('body'):
            if body.get('name') in ('mf_ds','rf_ds'):
                bodies.append(dict(asset=str(name),body=ET.tostring(body,encoding='unicode')))
    out=p.OUT/'proxy_identity';out.mkdir(exist_ok=True);m,manifest=build(env,out/'scene.xml',book_projection_mm=-10)
    proxies=[]
    for name in manifest['middle_ring_proxies']:
        g=m.geom(name).id;visual=[i for i in range(m.ngeom) if m.geom_bodyid[i]==m.geom_bodyid[g] and m.geom_group[i]==2 and m.geom_dataid[i]==m.geom_dataid[g]]
        proxies.append(dict(name=name,mesh=m.mesh(int(m.geom_dataid[g])).name,visual_ids=visual,position=m.geom_pos[g].tolist(),quaternion=m.geom_quat[g].tolist(),visual_positions=[m.geom_pos[i].tolist() for i in visual],visual_quaternions=[m.geom_quat[i].tolist() for i in visual]))
    report=dict(bodies=bodies,proxies=proxies,libraries={k:importlib.util.find_spec(k) is not None for k in ('trimesh','fcl','coacd','scipy')},devices_opened=False)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report),flush=True)
if __name__=='__main__':main()
