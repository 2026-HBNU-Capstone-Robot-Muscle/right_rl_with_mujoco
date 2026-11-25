"""Run stored OFF inputs and unchanged loaded guard replays; never hardware."""
import json
import subprocess
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261113/initial_empty_scope'
def wsl(p):return '/mnt/c/'+str(Path(p).resolve()).replace('\\','/')[3:]
jobs=[]
for label,stem,folder in [
    ('previous_support','support_first','support_first_trial_20261113_040308_069460'),
    ('latest_tip','tip_first','tip_first_trial_20261112_173936_071900')]:
    jobs.append((label,[ROOT/f'screen_{stem}_current_20261112.py',ROOT/folder/'fresh_off_request.json',OUT/label/'review.json']))
for stem,packet in [('support_first','support_first_trial'),('tip_first','tip_first_trial_v2')]:
    old=ROOT/'right_hand_rotation_20261112'/packet/'runtime_guard_review.json'
    saved=OUT/'before'/(stem+'_runtime_guard_review.json')
    if saved.exists():raise RuntimeError('Preserve guard replay backup')
    saved.write_bytes(old.read_bytes());old.unlink()
    jobs.append((stem+'_guards',[ROOT/f'screen_{stem}_runtime_guards_20261112.py']))
def run(job):
    name,args=job
    with (OUT/(name+'.log')).open('wb') as log:
        r=subprocess.run(['wsl.exe','--exec','/home/star1209042/venv/bin/python','-B',*[wsl(p) for p in args]],stdout=log,stderr=subprocess.STDOUT)
    print(json.dumps(dict(check=name,exit_code=r.returncode)),flush=True)
    return r.returncode
with ThreadPoolExecutor(max_workers=2) as pool:results=list(pool.map(run,jobs))
raise SystemExit(any(results))
