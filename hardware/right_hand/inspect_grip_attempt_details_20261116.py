import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
for name in ('final_grip_pose_20261116_155109_999585','final_grip_pose_20261116_155610_752108',
             'final_grip_pose_20261116_160542_994570','final_grip_pose_20261116_160804_734562'):
    folder=ROOT/name;r=json.loads((folder/'result.json').read_bytes())
    print('RUN',name,'START',r.get('preflight_state',{}).get('position'))
    print('COMMAND',r.get('last_issued_command'))
    images=sorted((folder/'camera').glob('*.jpg'))
    print('IMAGES',len(images),'LAST',str(images[-1]) if images else None)
    if r.get('samples'):
        s=r['samples'][-1];print('SAMPLE_KEYS',list(s));print('SAMPLE',json.dumps({k:s[k] for k in ('position','position_ticks','goal_ticks','goal','current_ma','tracking_rad') if k in s}))
    failures=r.get('read_failures',[])
    if failures: print('FIRST_FAILURES',json.dumps(failures[:2],ensure_ascii=False)[:2400])
    p=folder/'empty_recovery_screen/review.json'
    if p.exists():
        p=json.loads(p.read_bytes())
        print('STATIC',json.dumps(p['static_cases'][0],ensure_ascii=False))
        print('DYNAMIC',json.dumps(p['dynamic_cases'][0],ensure_ascii=False))
