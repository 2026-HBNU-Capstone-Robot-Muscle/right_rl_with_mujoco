"""Extend the independently verified small-turn working hold to30seconds."""
from pathlib import Path
import json,sys,time
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
import numpy as np,mujoco
from audit_opposed_working_grip_20261117 import MODEL,replay,sha

base=ROOT/'right_hand_rotation_20261117/opposed_lower_grip/core_support_chain_10s'
prior=json.loads((base/'review.json').read_bytes());old=np.load(base/'research_goals.npz',allow_pickle=False)
if not prior['working_grip_passed']:raise RuntimeError('Both support and upper space must already pass')
source=ROOT/prior['source_path']
if sha(source)!=prior['source_sha256'] or sha(MODEL)!=prior['model_sha256']:raise RuntimeError('Source changed')
rows=np.vstack([old['goals'],np.tile(old['goals'][-1],(266,1))]);hold_start=prior['hold_start']
if len(rows)-hold_start!=400:raise RuntimeError('Exactly30seconds required')
out=base/'extended_hold30';out.mkdir(exist_ok=False)
m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':MODEL.read_bytes()});began=time.perf_counter()
result=replay(m,dict(np.load(source,allow_pickle=False)),rows,hold_start,out,
    prior['audit']['final_center_world_m'] if 'final_center_world_m' in prior['audit'] else
    json.loads((ROOT/'right_hand_rotation_20261117/opposed_lower_grip'/
    'feedback_regrasp_normal1_gain1_nearpatch_basesupport_slip16_targetsurface_budget300_forceslide_margin2_paired_early141_latched_seated_freeifx/review.json').read_bytes())['hold_target_center_world_m'],
    json.loads((ROOT/'right_hand_rotation_20261117/opposed_lower_grip'/
    'feedback_regrasp_normal1_gain1_nearpatch_basesupport_slip16_targetsurface_budget300_forceslide_margin2_paired_early141_latched_seated_freeifx/review.json').read_bytes())['hold_target_rotation_world'],
    required_roles=('mf_tip','th_tip'))
np.savez_compressed(out/'research_goals.npz',start=old['start'],goals=rows,hold_start=hold_start)
report=dict(status='small_turn_regrasp_and30s_hold_uninterrupted',passed=result['passed'],audit=result,
    parent_review_sha256=sha(base/'review.json'),source_path=source.relative_to(ROOT).as_posix(),
    source_sha256=sha(source),model_sha256=sha(MODEL),goals=len(rows),hold_start=hold_start,
    appended_constant_goals=266,cube_or_hand_reset_between_segments=False,full_quarter_turn_cycle_verified=False,
    physical_grasp_verified=False,devices_opened=False,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began)
(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k not in ('timeline','certificate')},ensure_ascii=True),flush=True)
