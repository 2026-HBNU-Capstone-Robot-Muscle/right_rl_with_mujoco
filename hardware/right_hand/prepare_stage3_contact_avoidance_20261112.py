"""New comparison branch before a diagnosed IF/MF trap; preserve its failure."""
import json
import sys
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle'))
import mujoco
from native_cycle import assets, restore, Guard, BOUND, ticks_to_sim, save_json, sha

OUT=ROOT/'right_hand_rotation_20261112/stage3_preparation'
SOURCE=OUT/'relative_turn_contact_v2'
DEST=OUT/'relative_turn_contact_v3_start'
TARGET=ROOT/'stage3_contact_avoidance_teacher_20261112.py'
if DEST.exists() or TARGET.exists(): raise RuntimeError('Preserve comparison branch')
model,initial,_,_,_=assets('active_ccd50'); model.geom_friction[:,0]=.2
z=dict(np.load(SOURCE/'start_state.npz',allow_pickle=False)); data=restore(model,z); guard=Guard(model)
q=np.load(SOURCE/'research_goals.npz',allow_pickle=False)['goals'][:240]
for command in q:
    data.ctrl[:]=ticks_to_sim(command)
    for _ in range(75):
        mujoco.mj_step(model,data)
        if guard.check(data): raise RuntimeError('Original prefix replay failed')
DEST.mkdir()
np.savez_compressed(DEST/'start_state.npz',**{k:getattr(data,k) for k in BOUND},time=data.time,
                    up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
np.savez_compressed(DEST/'prefix_goals.npz',start=np.load(SOURCE/'research_goals.npz',allow_pickle=False)['start'],goals=q)
text=(ROOT/'stage3_fast_teacher_20261112.py').read_text(encoding='utf-8')
before="pair_names=[('mf_tip',f'palm_collision_{i}') for i in (1,4,6,8)]"
after=before+"\n    pair_names += [('if_md_collision_5','mf_px_collision')]"
if text.count(before)!=1: raise RuntimeError('Unexpected planning pair source')
text=text.replace(before,after,1)
before="ring_room=float(np.degrees(guard.qhi[9]-state.qpos[9]))"
after="residual=np.r_[residual,np.maximum(0.,.2-np.degrees(np.minimum(state.qpos[:16]-guard.qlo,guard.qhi-state.qpos[:16])))*5.]\n        "+before
if text.count(before)!=1: raise RuntimeError('Unexpected objective source')
text=text.replace(before,after,1)
compile(text,str(TARGET),'exec'); TARGET.write_text(text,encoding='utf-8')
save_json(DEST/'review.json',{'status':'new_branch_before_predicted_contact_not_old_success', 'replayed_prefix_goals':len(q),
          'source_goals_sha256':sha(SOURCE/'research_goals.npz'), 'source_state_sha256':sha(SOURCE/'start_state.npz'),
          'new_script_sha256':sha(TARGET),'old_failed_comparison_preserved':True,'every_executed_ms_checked':1,
          'native_guards_and_limits_changed':False,'new_terms_are_planning_costs_only':True,'devices_opened':0})
print('New branch prepared from an independently replayed 240-goal prefix; no native limits changed.')
