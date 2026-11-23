"""Check whether the existing index/thumb contacts can unload the middle."""
from pathlib import Path
import json,sys
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
import numpy as np,mujoco
from opposed_grip_force_balance_20261116 import allocate
from research_rotation_body_hold_20261104 import restore,inspect_for,native
from audit_opposed_working_grip_20261117 import MODEL,sha
BASE=ROOT/'right_hand_rotation_20261117/opposed_lower_grip'
source=BASE/'core_support_chain_10s/audit_end_state.npz'
m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':MODEL.read_bytes()})
d=restore(m,np.load(source,allow_pickle=False));hand,_,_=inspect_for(m);lo,hi,nlo,nhi=native(m);cube=m.geom('cube').id
cases=[]
for minimum in (.3,.6):
    control,result=allocate(m,d,cube,hand,nlo,nhi,lo,hi,tip_minimum=minimum,
        required_tips=('if_tip','th_tip'),normal_bounds={'mf_tip':(0.,.1)})
    cases.append(dict(required_min_N=minimum,feasible=control is not None,result=result))
report=dict(status='existing_contact_load_transfer_feasibility',source_sha256=sha(source),model_sha256=sha(MODEL),
    source_path=source.relative_to(ROOT).as_posix(),cases=cases,static_only=True,devices_opened=False,
    hardware_execution_allowed=False)
out=BASE/'support_transfer_feasibility';out.mkdir(exist_ok=False)
(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps(cases),flush=True)
