"""Preserve real native proof bytes and compare optimized replay with baseline."""
import hashlib
import json
import math
from pathlib import Path
from book_empty_preparation_plan import ROOT,BookPreparationPlan
from book_empty_preparation_gate import validate_proof

pointer=json.loads((ROOT/'book_screen_camera_validation_latest.json').read_bytes())
source=Path(pointer['folder']).resolve()
if not source.is_relative_to(ROOT):raise ValueError('Validation folder outside workspace')
report=json.loads((source/'result.json').read_bytes())
if (report['status']!='camera_and_full_native_screen_passed' or report['motor_interfaces_opened']
        or report['motor_commands_sent']!=0 or report['native_screen_seconds']>=120):
    raise ValueError('No completed bounded camera/native validation')
plan=BookPreparationPlan(source/'saved_off_request.json')
current=validate_proof(plan,source/'screen.json')
baseline_path=ROOT/'right_hand_rotation_20261024/book20_current_review/current_attempt_fixture/screen_v1.json'
baseline=json.loads(baseline_path.read_bytes())
if baseline['start_ticks']!=current['start_ticks'] or baseline['plan_sha256']!=current['plan_sha256']:
    raise ValueError('Baseline has different physical inputs or command path')
differences=[]
def compare(a,b,path):
    if type(a) is dict and type(b) is dict:
        if a.keys()!=b.keys():raise ValueError('Schema differs: '+path)
        for key in a:compare(a[key],b[key],path+'/'+key)
    elif type(a) is list and type(b) is list:
        if len(a)!=len(b):raise ValueError('Length differs: '+path)
        for i,(x,y) in enumerate(zip(a,b)):compare(x,y,path+'/'+str(i))
    elif isinstance(a,(int,float)) and not isinstance(a,bool) and isinstance(b,(int,float)) and not isinstance(b,bool):
        if not math.isfinite(a) or not math.isfinite(b) or abs(a-b)>1e-12:
            raise ValueError('Numerical result changed: '+path)
        differences.append(abs(a-b))
    elif a!=b:raise ValueError('Result differs: '+path)
for collection in ('static_screens','dynamic_sensitivity'):
    compare(baseline[collection],current[collection],collection)

destination=ROOT/'right_hand_rotation_20261024/book20_current_review/timeout_fix_fixture'
destination.mkdir(exist_ok=True)
for src,name in ((source/'saved_off_request.json','request.json'),(source/'screen.json','screen.json')):
    target=destination/name
    if target.exists() and target.read_bytes()!=src.read_bytes():raise ValueError('Refuse to overwrite an existing different proof')
    target.write_bytes(src.read_bytes())
validate_proof(BookPreparationPlan(destination/'request.json'),destination/'screen.json')
evidence=dict(status='same_full_replay_results_with_faster_camera_active_validation',
    original_timeout_seconds=120,native_screen_timeout_unchanged=True,
    camera_active_native_screen_seconds=report['native_screen_seconds'],
    native_worker_elapsed_seconds=current['elapsed_seconds'],
    baseline_numeric_results_compared=True,baseline_file=str(baseline_path),
    baseline_sha256=hashlib.sha256(baseline_path.read_bytes()).hexdigest(),
    static_rows_compared=len(current['static_screens']),dynamic_rows_compared=len(current['dynamic_sensitivity']),
    maximum_numeric_difference=max(differences,default=0.),
    source_validation_folder=str(source),camera_recording=report['camera_recording'],
    actual_proof_bytes_preserved=True,old_proof_hashes_modified=False,
    motor_interfaces_opened=False,motor_commands_sent=0)
(destination/'performance_validation.json').write_text(json.dumps(evidence,indent=2),encoding='utf-8')
print(json.dumps(evidence,indent=2))
