"""Audit a ten-second support subtask; upper-space failure remains a failure.

The replay concatenates the original hold, small turn and actual regrasp
commands, with no hand/cube resets. This never marks a working grasp ready.
"""
from pathlib import Path
import json,sys,time
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
import numpy as np,mujoco
from audit_opposed_working_grip_20261117 import MODEL,replay,sha

BASE=ROOT/'right_hand_rotation_20261117/opposed_lower_grip'
PARENT=BASE/'feedback_regrasp_normal1_gain1_nearpatch_basesupport_slip16_targetsurface_budget300_forceslide_margin2_paired_early141_latched_seated_freeifx'


def main():
    prior=json.loads((PARENT/'review.json').read_bytes())
    candidate=np.load(PARENT/'research_goals.npz',allow_pickle=False)
    seat=np.flatnonzero(candidate['phases']=='seat_middle')
    if not len(seat):raise RuntimeError('No acquired opposed-support stage')
    start=int(seat[0]);end=start+134
    if end>len(candidate['goals']):raise RuntimeError('Need full10.05s subtask')
    hold=BASE/'level_triple_normal1p2_settle1_settled_frame_active_hold400_heightlock0p0'
    first=json.loads((hold/'review.json').read_bytes());source=ROOT/first['source_path']
    if sha(source)!=first['source_sha256'] or sha(MODEL)!=prior['model_sha256']:raise RuntimeError('Source changed')
    turn=BASE/'next_turn5p0_palmclear_margin8_fingerclear_allpalms/research_goals.npz'
    h=np.load(hold/'research_goals.npz',allow_pickle=False)['goals']
    t=np.load(turn,allow_pickle=False)['goals'][:141]
    rows=np.vstack([h,t,candidate['goals'][:end]]);hold_start=len(h)+len(t)+start
    out=BASE/'core_support_chain_10s';out.mkdir(exist_ok=False)
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':MODEL.read_bytes()});began=time.perf_counter()
    result=replay(m,dict(np.load(source,allow_pickle=False)),rows,hold_start,out,
        prior['hold_target_center_world_m'],prior['hold_target_rotation_world'],
        required_roles=('mf_tip','th_tip'),core_diagnostic_only=True)
    np.savez_compressed(out/'research_goals.npz',start=np.load(hold/'research_goals.npz')['start'],goals=rows,hold_start=hold_start)
    protected=json.loads((ROOT/'right_hand_rotation_20261106/middle_clearance_probe/review_centered_v2.json').read_bytes())['code_hashes']
    if any(sha(ROOT/p)!=v for p,v in protected.items()):raise RuntimeError('Protected source changed')
    report=dict(status='uninterrupted_small_turn_and_opposed_support_diagnostic',target_turn_deg=5.,
        support_subtask_passed=result['core_support_passed'],working_grip_passed=result['passed'],
        audit=result,goals=len(rows),hold_start=hold_start,source_path=source.relative_to(ROOT).as_posix(),
        source_sha256=sha(source),model_sha256=sha(MODEL),regrasp_review_sha256=sha(PARENT/'review.json'),
        input_hashes={p.relative_to(ROOT).as_posix():sha(p) for p in [hold/'research_goals.npz',turn,PARENT/'research_goals.npz']},
        original_runtime_sources_unchanged=len(protected),cube_or_hand_reset_between_segments=False,
        full_quarter_turn_cycle_verified=False,physical_grasp_verified=False,devices_opened=False,
        hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('timeline','certificate')},ensure_ascii=True),flush=True)


if __name__=='__main__':main()
