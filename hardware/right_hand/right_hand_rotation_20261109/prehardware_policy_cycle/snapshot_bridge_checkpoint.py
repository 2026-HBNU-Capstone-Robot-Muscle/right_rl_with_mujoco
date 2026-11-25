"""Capture a mutually consistent executed-state/control checkpoint for a branch."""
from pathlib import Path
import argparse
import hashlib
import io
import json
import numpy as np


def digest(raw):return hashlib.sha256(raw).hexdigest()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--source',required=True);parser.add_argument('--tag',required=True);args=parser.parse_args()
    out=Path(__file__).resolve().parent;source=(out/args.source).resolve()
    if not source.is_relative_to(out) or not args.tag.replace('_','').isalnum():raise ValueError('Invalid local research path')
    folder=out/'next_cycle/handoff_refinement_20261110'/args.tag
    if folder.exists():raise RuntimeError('Preserve existing checkpoint snapshot')
    first=dict(np.load(source/'start_state.npz',allow_pickle=False));state=goals=None
    for _ in range(8):
        try:
            state_raw=(source/'checkpoint_state.npz').read_bytes();goals_raw=(source/'checkpoint_goals.npz').read_bytes()
            if state_raw!=(source/'checkpoint_state.npz').read_bytes():continue
            state=dict(np.load(io.BytesIO(state_raw),allow_pickle=False));goals=dict(np.load(io.BytesIO(goals_raw),allow_pickle=False))
            time_error=abs(float(state['time'])-float(first['time'])-len(goals['goals'])*.075)
            if time_error<1e-6:break
        except (OSError,ValueError,EOFError):state=goals=None
    else:raise RuntimeError('Checkpoint was being updated; no partial snapshot saved')
    if state is None or goals is None:raise RuntimeError('No complete checkpoint')
    folder.mkdir();(folder/'state.npz').write_bytes(state_raw);(folder/'goals.npz').write_bytes(goals_raw)
    source_code=source/'source_plan_clear_work_grip_bridge.py'
    (folder/'source_plan_clear_work_grip_bridge.py').write_bytes(source_code.read_bytes())
    report=dict(status='consistent_executed_model_checkpoint_snapshot',parent=source.relative_to(out).as_posix(),executed_goals=len(goals['goals']),
        model_time=float(state['time']),state_control_time_consistency_error_seconds=time_error,
        state_sha256=digest(state_raw),goals_sha256=digest(goals_raw),source_code_sha256=digest(source_code.read_bytes()),
        complete_parent_trial_or_task_not_claimed=True,not_a_proposed_future_state=True,devices_opened=0)
    (folder/'manifest.json').write_text(json.dumps(report,indent=2),encoding='utf-8');print(json.dumps(report),flush=True)


if __name__=='__main__':main()
