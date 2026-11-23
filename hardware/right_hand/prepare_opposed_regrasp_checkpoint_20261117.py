"""Replay the saved small turn only until it is ready to hand off to grasp."""
from pathlib import Path
import sys,json
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
import numpy as np,mujoco
from audit_opposed_working_grip_20261117 import MODEL,sha
from canonical_grip_dynamics_20261105 import run
from research_rotation_body_hold_20261104 import restore
from feedback_opposed_regrasp_20261117 import contacts

base=ROOT/'right_hand_rotation_20261117/opposed_lower_grip'
parent=base/'next_turn5p0_palmclear_margin8_fingerclear_allpalms'
prior=json.loads((parent/'review.json').read_bytes());source=ROOT/prior['source_path']
if sha(source)!=prior['source_sha256'] or sha(MODEL)!=prior['model_sha256']:raise RuntimeError('Source changed')
out=base/'regrasp_checkpoint_goal141';out.mkdir(exist_ok=False)
rows=np.load(parent/'research_goals.npz',allow_pickle=False)['goals'][:141]
m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':MODEL.read_bytes()})
z=dict(np.load(source,allow_pickle=False))
result=run(m,z,rows,1,out/'source.npz',None,False,5,('if_tip','mf_tip','th_tip'))
if not result['safety_passed'] or result['completed_goals']!=141:raise RuntimeError('Prefix replay failed')
d=restore(m,np.load(out/'source.npz',allow_pickle=False));forces,lower=contacts(m,d,m.geom('cube').id)
report=dict(status='earlier_small_turn_handoff_checkpoint',prefix_goals=141,
    source_path=source.relative_to(ROOT).as_posix(),source_sha256=sha(source),
    parent_goals_sha256=sha(parent/'research_goals.npz'),model_sha256=sha(MODEL),
    checkpoint_sha256=sha(out/'source.npz'),prefix_dynamic_result=result,
    contact_N=forces,opposed_lower_contact_N=lower,cube_reset_after_start=False,
    not_a_validated_grasp_or_hardware_ready_file=True,devices_opened=False)
(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(checkpoint=str(out/'source.npz'),contact_N=forces,lower=lower),ensure_ascii=True),flush=True)
