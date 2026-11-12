"""Preserve the latest stopped trial and compare its measured position to its proof."""
import hashlib,json,shutil
from pathlib import Path
import numpy as np
from recovered_forward_plan_20261026 import RecoveredForwardPlan,ROOT
from thumb_mcp_clearance_guard_20261026 import policy
SOURCE=ROOT/'recovered_forward_placement_20261026_034826_275789'
OUT=ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/mcp_checkpoint_stop_0348'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    OUT.mkdir(exist_ok=True);r=json.loads((SOURCE/'result.json').read_bytes());p=RecoveredForwardPlan(SOURCE/'fresh_off_request.json');proof=json.loads((SOURCE/'current_start_screen.json').read_bytes())
    if proof['source_sha256']!=p.SOURCE_SHA or proof['plan_sha256']!=p.plan_digest(p.source_position()):raise ValueError('Historical target proof mismatch')
    q=policy(p,proof);s=r['samples'][-1];actual=s['actual'][1];goal=s['goal'][1]
    report=dict(source=SOURCE.name,source_sha256=sha(SOURCE/'result.json'),proof_sha256=sha(SOURCE/'current_start_screen.json'),
        error=r['error'],actual_mcp_ticks=actual,command_mcp_ticks=goal,lag_ticks=actual-goal,
        lag_deg=(actual-goal)*360/4096,old_limit_ticks=20,over_old_limit_ticks=actual-goal-20,
        new_clearance_policy=q,measured_position_already_past_release=bool(q['threshold_ticks'] is None or actual<=q['threshold_ticks']),
        max_tracking_rad=max(max(v['tracking_rad']) for v in r['samples']),peak_current_ma=int(max(np.max(abs(np.asarray(v['current_ma']))) for v in r['samples'])),
        min_voltage_v=min(min(v['voltage_v']) for v in r['samples']),camera_usable_frames=r['camera_recording']['image_signal_usable_frames'],
        camera_blank_frames=r['camera_recording']['image_signal_blank_frames'],all_torque_off_verified=r['all_torque_off_verified'],settings_restored=r['settings_restored'],
        cube_placement_reached=False,physical_cube_rotation_verified=False)
    (OUT/'review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    archive=OUT/'code_before_checkpoint_fix';archive.mkdir(exist_ok=True)
    for name in ('recovered_forward_plan_20261026.py','leap_recovered_forward_placement_20261026.py','interrupted_thumb_contact_policy_20261026.py'):
        target=archive/name
        if target.exists() and target.read_bytes()!=(ROOT/name).read_bytes():raise ValueError('Refuse to overwrite different archived code')
        if not target.exists():shutil.copyfile(ROOT/name,target)
    print(json.dumps({k:v for k,v in report.items() if k!='new_clearance_policy'},ensure_ascii=False))
    print('New measured release threshold:',q['threshold_ticks'])
if __name__=='__main__':main()
