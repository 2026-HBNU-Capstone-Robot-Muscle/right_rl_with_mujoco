"""Bounded saved-file review. No devices and no raw-report modifications."""
import argparse,collections,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
parser=argparse.ArgumentParser();parser.add_argument('--run',required=True);args=parser.parse_args()
folder=(ROOT/args.run).resolve();folder.relative_to(ROOT)
r=json.loads((folder/'result.json').read_bytes())
small={k:v for k,v in r.items() if not isinstance(v,(list,dict))}
small['phases']=dict(collections.Counter(s['phase'] for s in r.get('samples',[])))
small['read_failures']=r.get('read_failures',[])
small['photo_metadata']={k:v for k,v in r.items() if 'photo' in k}
frame_path=folder/'camera/frames.json';frames=json.loads(frame_path.read_bytes())['frames'] if frame_path.exists() else []
small['frames']=dict(count=len(frames),bad=sum(f.get('image_signal_usable') is not True for f in frames),first=frames[0] if frames else None,last=frames[-1] if frames else None)
print(json.dumps(small))
if r.get('status')=='calibrated_quarter_observation_complete':
    stamp=folder.name.rsplit('_',2)[1];out=ROOT/f'right_hand_rotation_20261028/completed_calibrated_{stamp}';out.mkdir(exist_ok=True)
    for source,stem in (('measure_same_marker_045326_20261028.py','measure_same_marker_calibrated'),('trace_observed_rotation_045326_20261028.py','trace_calibrated'),('export_real_quarter_045326_20261028.py','export_calibrated')):
        code=(ROOT/source).read_text(encoding='utf-8').replace('horizontal_quarter_trial_20261028_045326_636152',folder.name).replace('completed_quarter_045326',out.name)
        target=ROOT/f'{stem}_{stamp}_20261028.py'
        if target.exists():raise RuntimeError('Refuse to overwrite existing analysis entry')
        target.write_bytes(code.encode())
    print('Independent saved-data analysis entries created. No devices opened.')
