"""Replay hold -> small turn -> regrasp -> hold as one uninterrupted stream."""
from pathlib import Path
import argparse,json,sys,time
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
import numpy as np,mujoco
from audit_opposed_working_grip_20261117 import MODEL,replay,sha

BASE=ROOT/'right_hand_rotation_20261117/opposed_lower_grip'


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--regrasp',required=True);a=ap.parse_args()
    parent=(BASE/a.regrasp).resolve();parent.relative_to(BASE.resolve())
    current=json.loads((parent/'review.json').read_bytes())
    if not current['regrasp_and_hold_passed']:raise RuntimeError('Completed independent regrasp/hold required')
    checkpoint=BASE/'regrasp_checkpoint_goal141/source.npz'
    if sha(checkpoint)!=current['source_sha256']:raise RuntimeError('Expected early-turn checkpoint')
    hold=BASE/'level_triple_normal1p2_settle1_settled_frame_active_hold400_heightlock0p0'
    first=json.loads((hold/'review.json').read_bytes());source=ROOT/first['source_path']
    if sha(source)!=first['source_sha256'] or sha(MODEL)!=current['model_sha256']:raise RuntimeError('Source changed')
    turn=BASE/'next_turn5p0_palmclear_margin8_fingerclear_allpalms/research_goals.npz'
    files=[hold/'research_goals.npz',turn,parent/'research_goals.npz']
    arrays=[np.load(p,allow_pickle=False)['goals'] for p in files];arrays[1]=arrays[1][:141]
    rows=np.vstack(arrays);hold_start=len(arrays[0])+len(arrays[1])+current['hold_start']
    out=parent/'continuous_early141_chain';out.mkdir(exist_ok=False)
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':MODEL.read_bytes()})
    began=time.perf_counter();result=replay(m,dict(np.load(source,allow_pickle=False)),rows,hold_start,out,
        current['hold_target_center_world_m'],current['hold_target_rotation_world'],required_roles=('mf_tip','th_tip'))
    np.savez_compressed(out/'research_goals.npz',start=np.load(files[0],allow_pickle=False)['start'],goals=rows,hold_start=hold_start)
    protected=json.loads((ROOT/'right_hand_rotation_20261106/middle_clearance_probe/review_centered_v2.json').read_bytes())['code_hashes']
    changed=[p for p,h in protected.items() if sha(ROOT/p)!=h]
    if changed:raise RuntimeError('Protected files changed: '+repr(changed))
    report=dict(status='uninterrupted_small_turn_regrasp_chain',small_angle_chain_passed=result['passed'],
        target_turn_deg=5.,source_path=source.relative_to(ROOT).as_posix(),source_sha256=sha(source),
        model_sha256=sha(MODEL),input_goals_sha256={p.relative_to(ROOT).as_posix():sha(p) for p in files},
        regrasp_review_sha256=sha(parent/'review.json'),goals=len(rows),hold_start=hold_start,audit=result,
        cube_or_hand_reset_between_segments=False,frozen_sources_unchanged=len(protected),
        full_quarter_turn_cycle_verified=False,physical_grasp_verified=False,devices_opened=False,
        hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('timeline','certificate')},ensure_ascii=True),flush=True)


if __name__=='__main__':main()
