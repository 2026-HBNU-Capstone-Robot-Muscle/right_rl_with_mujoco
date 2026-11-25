"""Freeze a mutually consistent running teacher checkpoint for another branch."""
from pathlib import Path
import json
import hashlib
import numpy as np
OUT=Path(__file__).resolve().parent;BASE=OUT/'next_cycle/handoff_refinement_20261110'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    parent=BASE/'robust_mu025_late_whole_hand_v19';source=BASE/'robust_mu025_feedback_pressure_v12/force1.5/end_state.npz';folder=BASE/'late_teacher_checkpoint_for_contact_floor'
    if folder.exists():raise RuntimeError('Preserve frozen checkpoint')
    r=json.loads((parent/'progress.json').read_text());state_bytes=(parent/'checkpoint_state.npz').read_bytes();goals_bytes=(parent/'checkpoint_goals.npz').read_bytes()
    import io
    state=np.load(io.BytesIO(state_bytes),allow_pickle=False);goals=np.load(io.BytesIO(goals_bytes),allow_pickle=False);start=np.load(source,allow_pickle=False)
    assert len(goals['goals'])==r['step'] and abs(float(state['time'])-float(start['time'])-r['step']*.075)<1e-6,'Checkpoint crossed an iteration; retry the read'
    assert all(np.isfinite(state[k]).all() for k in ('qpos','qvel','ctrl','act','qacc_warmstart'))
    folder.mkdir();(folder/'end_state.npz').write_bytes(state_bytes);(folder/'research_goals.npz').write_bytes(goals_bytes)
    report=dict(status='frozen_consistent_actual_teacher_checkpoint',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),parent=parent.relative_to(OUT).as_posix(),steps=r['step'],actual=r['actual'],end_state_sha256=sha(folder/'end_state.npz'),goals_sha256=sha(folder/'research_goals.npz'),checkpoint_is_actual_dynamic_branch_not_teleported_pose=True,full_initial_placement_replay_not_yet_performed=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False)
    (folder/'review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(dict(steps=r['step'],error_deg=r['actual']['error_deg'])))

if __name__=='__main__':main()
