"""Remove isolated projection branch jumps only after independent chord checks."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,hashlib
from pathlib import Path
import numpy as np
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from collision_projection_20261025 import CollisionProjection
from hand_motor_mapping import sim_to_ticks,ticks_to_sim
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/corrected_policy_reference'

def main():
    p=OUT/'projected_policy_reference.json';blob=p.read_bytes();old=json.loads(blob)
    teacher=np.asarray(old['teacher_sim_goals_rad']);path=list(np.asarray(old['projected_goals_sim_rad']));ts=list(old['sparse_source_parameters'])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build(env,OUT/'bridged_policy_CAD.xml');projector=CollisionProjection(m)
    def chord(a,b,ta,tb):
        a,b=sim_to_ticks(a),sim_to_ticks(b);n=max(1,int(np.ceil(np.max(abs(b-a))/2)))
        rows=ticks_to_sim(np.rint(a+(b-a)*np.arange(n+1)[:,None]/n).astype(int))
        params=np.linspace(ta,tb,n+1)
        refs=np.array([np.interp(params,np.arange(len(teacher)),teacher[:,j]) for j in range(16)]).T
        if np.max(abs(rows-refs))>.451:return False
        return all(projector.maximum_overlap(q)<1e-7 for q in rows)
    removed=[]
    for iteration in range(8):
        blocked=[i for i in range(len(path)-1) if not chord(path[i],path[i+1],ts[i],ts[i+1])]
        if not blocked:break
        changed=False
        for i in sorted(set(blocked+[i+1 for i in blocked])):
            if i<=0 or i>=len(path)-1:continue
            if chord(path[i-1],path[i+1],ts[i-1],ts[i+1]):
                removed.append(ts[i]);path.pop(i);ts.pop(i);changed=True;break
        print('BRIDGE_POLICY',iteration,'blocked',len(blocked),'removed',removed[-1:] if changed else [],flush=True)
        if not changed:break
    dense=[sim_to_ticks(path[0])];dense_t=[ts[0]]
    for a,b,ta,tb in zip(path[:-1],path[1:],ts[:-1],ts[1:]):
        a,b=sim_to_ticks(a),sim_to_ticks(b);n=max(1,int(np.ceil(np.max(abs(b-a))/2)))
        for k in range(1,n+1):dense.append(np.rint(a+(b-a)*k/n).astype(int));dense_t.append(ta+(tb-ta)*k/n)
    maximum=max(projector.maximum_overlap(q) for q in ticks_to_sim(np.array(dense)))*1000
    result={**old,'parent_projection_sha256':hashlib.sha256(blob).hexdigest(),
        'projected_goals_sim_rad':np.array(path).tolist(),'sparse_source_parameters':ts,
        'dense_raw_ticks':np.array(dense).tolist(),'dense_source_parameters':dense_t,
        'removed_isolated_reference_parameters':removed,'maximum_dense_model_overlap_mm':maximum,
        'full_static_path_clear':maximum<.0001,'model':manifest,'projection_revision':2}
    (OUT/'projected_policy_reference_v2.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('BRIDGE_COMPLETE','clear',result['full_static_path_clear'],'overlap',maximum,'removed',removed,flush=True)

if __name__=='__main__':main()
