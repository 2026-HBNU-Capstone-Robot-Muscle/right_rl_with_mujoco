"""Independently replay a fixed20.025s postturn hold from the original source."""
from pathlib import Path
import json, sys, time
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
import mujoco
import numpy as np
from audit_opposed_working_grip_20261117 import MODEL, replay, sha

base = ROOT/'right_hand_rotation_20261117/opposed_lower_grip'
parent = base/'core_support_chain_10s'
predictive = parent/'predictive_hold30_h3_edge0.5_guarded'
out = parent/'predictive_verified20s'
out.mkdir(exist_ok=False)
prior = json.loads((parent/'review.json').read_bytes())
generation = json.loads((predictive/'review.json').read_bytes())
source = ROOT/prior['source_path']
if sha(source) != prior['source_sha256'] or sha(MODEL) != prior['model_sha256']:
    raise RuntimeError('Original source/model changed')
if sha(parent/'audit_end_state.npz') != generation['source_sha256']:
    raise RuntimeError('Predictive continuation source changed')
old = np.load(parent/'research_goals.npz', allow_pickle=False)
tail = np.load(predictive/'suffix.npz', allow_pickle=False)
if len(tail['goals']) < 133 or generation['stop']['goal'] <= 133:
    raise RuntimeError('Insufficient completed prediction prefix')
rows = np.vstack([old['goals'], tail['goals'][:133]])
target = json.loads((base/'feedback_regrasp_normal1_gain1_nearpatch_basesupport_slip16_targetsurface_budget300_forceslide_margin2_paired_early141_latched_seated_freeifx/review.json').read_bytes())
m = mujoco.MjModel.from_binary_path('model.mjb', assets={'model.mjb': MODEL.read_bytes()})
began = time.perf_counter()
audit = replay(m, dict(np.load(source, allow_pickle=False)), rows, prior['hold_start'], out,
    target['hold_target_center_world_m'], target['hold_target_rotation_world'], required_roles=('mf_tip', 'th_tip'))
np.savez_compressed(out/'research_goals.npz', start=old['start'], goals=rows, hold_start=prior['hold_start'])
result = dict(status='independent_full_prefix_small_turn_predictive20s_hold', passed=audit['passed'], audit=audit,
    goals=len(rows), hold_start=prior['hold_start'], appended_goals=133,
    source_path=source.relative_to(ROOT).as_posix(), source_sha256=sha(source), model_sha256=sha(MODEL),
    generation_review_sha256=sha(predictive/'review.json'), goals_sha256=sha(out/'research_goals.npz'),
    validator_sha256=sha(ROOT/'audit_opposed_working_grip_20261117.py'),
    cube_or_hand_reset_between_segments=False, accepted_cube_pose_resets=0,
    longer30s_generation_failed=True, cropped_before_failure=True,
    full_quarter_turn_cycle_verified=False, physical_grasp_verified=False,
    devices_opened=False, hardware_execution_allowed=False, elapsed_seconds=time.perf_counter()-began)
(out/'review.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
print(json.dumps({k:v for k,v in audit.items() if k not in ('timeline', 'certificate')}, ensure_ascii=True), flush=True)
