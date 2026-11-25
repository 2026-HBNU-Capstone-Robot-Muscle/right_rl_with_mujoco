"""Static search for a bounded EMPTY tip/PIP separation, unchanged checks."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import mujoco
import numpy as np
from mujoco_playground import registry
from hand_motor_mapping import ticks_to_sim
from initial_contact_escape import ContactEscape
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from slow_grasp_demo_plan import SlowGraspDemoPlan
from grasp_recovery_plan import THUMB_NATIVE_RECOVERY_FLOOR

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'right_hand_rotation_20261023/slow_grasp_demo_fixture'


def main():
    raw=np.array(json.loads((FOLDER/'fresh_off_request.json').read_bytes())['off_state']['position'])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,_=build_with_middle_thumb_hulls(env);d=mujoco.MjData(m)
    cube=m.geom('cube').id;m.geom_contype[cube]=m.geom_conaffinity[cube]=0
    hand={m.body('palm').id}
    for i in range(m.nbody):
        if int(m.body_parentid[i]) in hand:hand.add(i)
    def contacts(q):
        d.qpos[:16]=ticks_to_sim(q);d.qpos[-7:]=[0,0,2,1,0,0,0];mujoco.mj_forward(m,d)
        result={}
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2)
            if int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand and c.dist<0:
                key='|'.join(sorted((m.geom(a).name,m.geom(b).name)))
                result[key]=max(result.get(key,0.),-float(c.dist)*1000)
        return result
    offsets=[np.zeros(16,dtype=int),np.full(16,-4),np.full(16,4)]
    for i in range(16):
        for sign in (-1,1):v=np.zeros(16,dtype=int);v[i]=sign*4;offsets.append(v)
    offsets.extend(np.random.default_rng(20261021).integers(-4,5,(32,16)))
    results=[]
    for extension in (64,72,80):
        passed=[];failures=[]
        for off in offsets:
            start=raw+off;target=start.copy();target[14]=2625;target[3]=max(start[3],THUMB_NATIVE_RECOVERY_FLOOR)
            target[12]-=extension
            n=max(20,int(np.ceil(max(abs(target-start))/2)))
            escape=ContactEscape(contacts(start),SlowGraspDemoPlan.INITIAL_SELF_ESCAPE_PAIRS,None,40,monotonic=True)
            for step in range(1,n+1):
                q=np.rint(start+(target-start)*step/n).astype(int)
                q[12]=start[12]+int(np.clip(-extension,-2*step,2*step))
                escape.observe(contacts(q),step)
            r=escape.result();good=r['self_escape_complete'] and r['self_escape_max_unapproved_overlap_mm']<=1e-6
            passed.append(bool(good))
            if not good:failures.append(dict(offset=off.tolist(),**r))
        results.append(dict(tip_extension_ticks=extension,passed=sum(passed),cases=len(passed),failures=failures))
        print('EXTENSION',extension,'PASSED',sum(passed),'/',len(passed),flush=True)
    (FOLDER/'ring_release_static_search.json').write_text(json.dumps(dict(results=results,
        hardware_execution_allowed=False,static_only=True,no_control_thresholds_changed=True),indent=2),encoding='utf-8')


if __name__=='__main__':main()
