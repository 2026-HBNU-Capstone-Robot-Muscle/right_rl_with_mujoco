"""Static model frame metadata only; no device interface."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np,mujoco
from mujoco_playground import registry
ROOT=Path(__file__).resolve().parent
def main():
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m=env.mj_model;d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    old=ET.parse(ROOT.parent/'work/Leap_Test/train/env/xmls/leap_rh_mjx.xml');oldp=next(v for v in old.iter('body') if v.get('name')=='palm');bodies={v.get('name'):v for v in oldp if v.tag=='body'}
    out=dict(palm_world_position=d.xpos[m.body('palm').id].tolist(),palm_world_rotation=d.xmat[m.body('palm').id].reshape(3,3).tolist(),
        native_child_positions={name:m.body_pos[m.body(name).id].tolist() for name in ('if_bs','mf_bs','rf_bs','th_mp')},
        legacy_child_positions={name:np.fromstring(bodies[name].get('pos'),sep=' ').tolist() for name in ('if_bs','mf_bs','rf_bs','th_mp')},
        legacy_palm_position=oldp.get('pos'),legacy_palm_quaternion=oldp.get('quat'),hardware_execution_allowed=False,real_devices_opened=False)
    path=ROOT/'right_hand_rotation_20261026/saved_marker_registration/model_coordinate_frames.json';path.write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8');print(json.dumps(out),flush=True)
if __name__=='__main__':main()
