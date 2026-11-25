"""Read the completed shorter-route trial and prepare isolated analysis copies."""
import collections,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
folder=ROOT/'calibrated_quarter_trial_20261028_052456_066430'
r=json.loads((folder/'result.json').read_bytes())
print(json.dumps({k:v for k,v in r.items() if not isinstance(v,(dict,list))}))
print('PHASES',json.dumps(dict(collections.Counter(s['phase'] for s in r.get('samples',[])))))
print('PHOTO_METADATA',json.dumps({k:v for k,v in r.items() if 'photo' in k}))
print('SIGNAL',json.dumps(r.get('camera_image_signal')))
print('READ_FAILURES',json.dumps(r.get('read_failures',[])))
frames=json.loads((folder/'camera/frames.json').read_bytes())['frames']
print('FRAMES',json.dumps(dict(count=len(frames),first=frames[0],last=frames[-1])))
out=ROOT/'right_hand_rotation_20261028/completed_calibrated_052456';out.mkdir(exist_ok=True)
for source,target in (
    ('measure_same_marker_045326_20261028.py','measure_same_marker_calibrated_052456_20261028.py'),
    ('trace_observed_rotation_045326_20261028.py','trace_calibrated_052456_20261028.py'),
    ('export_real_quarter_045326_20261028.py','export_calibrated_052456_20261028.py')):
    code=(ROOT/source).read_text(encoding='utf-8')
    assert 'horizontal_quarter_trial_20261028_045326_636152' in code
    code=code.replace('horizontal_quarter_trial_20261028_045326_636152',folder.name)
    code=code.replace('completed_quarter_045326',out.name)
    (ROOT/target).write_bytes(code.encode())
print('Independent analysis entries created; historical proof sources unchanged. No devices opened.')
