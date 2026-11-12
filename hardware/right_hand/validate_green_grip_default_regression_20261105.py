"""Replay the unchanged GREEN-up default and compare with saved golden data."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import json,hashlib,math
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT,R
from canonical_grip_dynamics_20261105 import run
folder=ROOT/'full_placement_v3_thumb12/front-10';source=folder/'review.json';raw=json.loads(source.read_bytes());gold=next(c for c in raw['cases'] if (c['gain_scale'],c['friction'])==(1.,.3))['continuous'];m=mujoco.MjModel.from_binary_path(str(folder/'g1_f0.3/model.mjb'));z=np.load(folder/'g1_f0.3/initial_state.npz');path=np.load(R/raw['candidate']);actual=run(m,z,path['goals'],1417,None,None,True);differences=[];compared=0
def compare(a,b,prefix):
    global compared
    if isinstance(a,dict):
        for k,v in a.items():compare(v,b[k],prefix+'.'+k)
    elif isinstance(a,list):
        if len(a)!=len(b):differences.append(prefix+' length')
        else:
            for i,(u,v) in enumerate(zip(a,b)):compare(u,v,prefix+f'[{i}]')
    elif isinstance(a,(int,float)) and not isinstance(a,bool):
        compared+=1
        if not math.isclose(a,b,abs_tol=2e-10,rel_tol=1e-12):differences.append(dict(field=prefix,old=a,new=b))
    else:
        compared+=1
        if a!=b:differences.append(dict(field=prefix,old=a,new=b))
compare(gold,actual,'continuous')
report=dict(status='unchanged_default_face5_full_path_regression',source=str(source.relative_to(R)),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),candidate_sha256=raw['candidate_sha256'],goals=len(path['goals']),compared_scalar_fields=compared,differences=differences,passed=not differences,source_reports_and_states_not_modified=True,devices_opened=False,hardware_execution_allowed=False)
(ROOT/'green_default_regression.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report),flush=True)
if differences:raise SystemExit(1)
