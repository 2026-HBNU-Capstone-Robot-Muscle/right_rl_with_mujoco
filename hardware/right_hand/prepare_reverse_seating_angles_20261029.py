import hashlib,json,math
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent;O=ROOT/'right_hand_rotation_20261029/reverse_cube_scout'
source=O/'forward_endpoint/review.json';r=json.loads(source.read_bytes());c=r['cases'][0];q=np.asarray(c['initial_cube_qpos'][3:])
for deg in (75,83,90):
    theta=math.radians(deg)*.5;b=np.array([math.cos(theta),0.,-math.sin(theta),0.]);product=np.r_[q[0]*b[0]-q[1:]@b[1:],q[0]*b[1:]+b[0]*q[1:]+np.cross(q[1:],b[1:])];product/=np.linalg.norm(product)
    path=O/f'seated_angle{deg}_seed.json'
    if path.exists():raise RuntimeError('Refuse to overwrite seed')
    path.write_text(json.dumps(dict(cube_qpos=c['final_cube_qpos'][:3]+product.tolist(),source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        initial_local_green_axis_turn_deg=deg,vertical_seating_required=True,hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
