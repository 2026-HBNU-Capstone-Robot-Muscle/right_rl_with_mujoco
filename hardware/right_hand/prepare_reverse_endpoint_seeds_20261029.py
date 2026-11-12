import hashlib,json,math
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent;O=ROOT/'right_hand_rotation_20261029/reverse_cube_scout'
source=O/'forward_endpoint/review.json';r=json.loads(source.read_bytes());c=r['cases'][0]
assert r['all_passed'] and len(r['cases'])==1
q=np.asarray(c['initial_cube_qpos'][3:]);b=np.array([math.sqrt(.5),0.,-math.sqrt(.5),0.])
product=np.r_[q[0]*b[0]-q[1:]@b[1:],q[0]*b[1:]+b[0]*q[1:]+np.cross(q[1:],b[1:])];product/=np.linalg.norm(product)
for name,pose in (('natural_endpoint',c['final_cube_qpos']),('endpoint_center_quarter_label',c['final_cube_qpos'][:3]+product.tolist())):
    path=O/(name+'_seed.json')
    if path.exists():raise RuntimeError('Refuse to overwrite seed')
    path.write_text(json.dumps(dict(cube_qpos=pose,source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        construction=name,hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
print('Two initial placement hypotheses saved; no device access.')
