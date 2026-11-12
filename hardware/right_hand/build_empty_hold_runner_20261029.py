"""Copy the existing checked runner, exposing its original held-joint guard."""
import ast,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parent
source=ROOT/'horizontal_quarter_motion_runner_20261027.py';data=source.read_text(encoding='utf-8')
anchor="        progress_history={};progress_guard=GoalAwareProgress();report['progress_policy']=progress_metadata();paused_seconds=0.;checkpoint_done=False;release_done=False"
assert data.count(anchor)==1
changed=data.replace(anchor,'        bus.empty_hold_guard=held_guard\n'+anchor)
dest=ROOT/'empty_hold_motion_runner_20261029.py'
dest.write_text(changed,encoding='utf-8')
print('Runner copied; only original guard exposure added.',hashlib.sha256(source.read_bytes()).hexdigest())
