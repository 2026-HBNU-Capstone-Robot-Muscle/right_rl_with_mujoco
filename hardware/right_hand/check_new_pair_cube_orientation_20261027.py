import json,numpy as np
from new_pair_grasp_plan_20261027 import SEED,OUT
def rotate(q):
    w,x,y,z=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
seed=json.loads(SEED.read_bytes());r=json.loads((OUT/'review.json').read_bytes());v=next(v for v in r['loaded_physics'] if v['book_front_mm']==-10 and v['gain_scale']==1 and v['friction']==.3)
for name,q in [('seed',seed['settled_cube_qpos']),('settled',v['settled_cube_qpos']),('after_open',v['final_cube_qpos'])]:
    normal=rotate(q[3:])@np.array(seed['up_cube_normal']);angle=np.degrees(np.arccos(np.clip(np.dot(normal,seed['up_world']),-1,1)));print(name,'green_up_tilt_deg',round(float(angle),3),'xyz',q[:3])
