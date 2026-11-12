import os
os.environ['MUJOCO_GL']='egl'
os.environ['JAX_PLATFORMS']='cpu'
import json,numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT
from research_rotation_body_hold_20261104 import inspect_for
m=mujoco.MjModel.from_binary_path(str(ROOT/'full_placement/front-10/g1_f0.3/model.mjb'));hand,_,_=inspect_for(m);rows=[]
for g in range(m.ngeom):
    b=int(m.geom_bodyid[g]);name=m.body(b).name
    if b not in hand and not name.startswith(('support','floor','leap_mount')):rows.append(dict(geom=m.geom(g).name,body=name,mocap=int(m.body_mocapid[b]),group=int(m.geom_group[g]),size=m.geom_size[g].tolist(),type=int(m.geom_type[g])))
test=mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom type="cylinder" size=".02 .01"/></worldbody></mujoco>');d=mujoco.MjData(test);mujoco.mj_forward(test,d);scene=mujoco.MjvScene(test,maxgeom=100);camera=mujoco.MjvCamera();opt=mujoco.MjvOption();mujoco.mjv_updateScene(test,d,opt,None,camera,mujoco.mjtCatBit.mjCAT_ALL,scene)
custom=mujoco.MjvGeom();mujoco.mjv_initGeom(custom,mujoco.mjtGeom.mjGEOM_CYLINDER,np.array([.02,.02,.01]),np.zeros(3),np.eye(3).reshape(9),np.ones(4,np.float32))
print(json.dumps(dict(nonhand_geoms=rows,native_cylinder_size=test.geom_size[0].tolist(),visual_cylinder_size=scene.geoms[0].size.tolist(),initGeom_passed_02_02_01_result_size=custom.size.tolist())),flush=True)
