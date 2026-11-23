"""Three independent nominal model cases; no devices."""
import os,signal,sys,subprocess,json,concurrent.futures
from pathlib import Path
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261116/cube_grasp_hold'
for entry in Path('/proc').iterdir():
    if not entry.name.isdigit() or int(entry.name)==os.getpid():continue
    try:args=(entry/'cmdline').read_bytes().split(b'\0')
    except OSError:continue
    if len(args)>2 and args[2].split(b'/')[-1]==b'screen_cube_grasp_loaded_20261116.py' and b'--front' not in args:
        os.kill(int(entry.name),signal.SIGTERM)
def check(front):
    dest=OUT/f'loaded_nominal_{front}';dest.mkdir(exist_ok=True)
    cmd=[sys.executable,'-B',str(ROOT/'screen_cube_grasp_loaded_20261116.py'),
        '--candidate',str((OUT/'candidate.npz').relative_to(ROOT)),
        '--output-folder',str(dest.relative_to(ROOT)), '--front',str(front),'--gain','1','--friction','.3',
        '--cube-seed-json','right_hand_rotation_20261030/stable_support_research/natural_recess_hypothesis_seed.json','--seat-up']
    with (dest/'check.log').open('w') as log:subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,check=True,cwd=ROOT)
    r=json.loads((dest/'review.json').read_bytes());print('FRONT',front,r['all_passed'],flush=True);return r
with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:reports=list(pool.map(check,(-15,-10,-5)))
r=dict(reports[0]);r['cases']=[c for report in reports for c in report['cases']]
r['all_passed']=all(c['passed'] for c in r['cases']);r['scope']='30 seconds additional hold, nominal gain/friction, three support fronts; prior27 safety cases remain separately bound'
(OUT/'loaded').mkdir(exist_ok=True);(OUT/'loaded/review.json').write_text(json.dumps(r,indent=2)+'\n')
print('DONE',r['all_passed'],flush=True)
