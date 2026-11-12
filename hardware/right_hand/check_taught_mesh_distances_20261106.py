"""Check distance-query outputs against stored native contacts."""
import json
import numpy as np,mujoco
from research_taught_grasp_20261106 import O
from research_rotation_body_hold_20261104 import restore
def main():
    m=mujoco.MjModel.from_binary_path(str(O/'model.mjb'));cube=m.geom('cube').id
    for path in [O/'baseline_end.npz',O/'sliding_contact/shape1_height3/middle_first_end.npz',O/'dynamic_refinement/end.npz']:
        d=restore(m,np.load(path));r=[]
        for name in ('mf_tip','th_tip'):
            g=m.geom(name).id;p=np.full(6,np.nan);dist=mujoco.mj_geomDistance(m,d,cube,g,.15,p)
            r.append(dict(name=name,distance_mm=float(dist*1000),closest_points=p.tolist(),native_contacts=[dict(distance_mm=float(c.dist*1000),point=c.pos.tolist()) for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,g}],contype=[int(m.geom_contype[cube]),int(m.geom_contype[g])],affinity=[int(m.geom_conaffinity[cube]),int(m.geom_conaffinity[g])]))
        print(json.dumps(dict(source=path.relative_to(O).as_posix(),queries=r)),flush=True)
if __name__=='__main__':main()
