"""Saved-start comparison and compact static failure evidence; no devices."""
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'thumb_clear_root_empty_20261026_123524_339920'
def main():
    r=json.loads((FOLDER/'result.json').read_bytes());request=json.loads((FOLDER/'fresh_off_request.json').read_bytes())
    old=json.loads((ROOT/'finish_empty_readonly_20261026_120039_495713/result.json').read_bytes())['snapshots'][-1]
    a=np.asarray(request['off_state']['position']);print('START',a.tolist(),'DELTA',(a-old['position']).tolist())
    print('RUN',json.dumps({k:r.get(k) for k in ('error','all_torque_off_verified','settings_restored','camera_recording')},ensure_ascii=False), 'WRITES',len(r.get('write_events',[])))
    proof=json.loads((FOLDER/'current_start_screen.json').read_bytes());bad=[]
    for s in proof['static_screens']:
        if s['self_escape_max_unapproved_overlap_mm']>1e-6 or not s['self_escape_complete']:
            bad.append({k:s.get(k) for k in ('book_front_mm','offset_ticks','initial_self_contacts_mm','self_escape_max_unapproved_overlap_mm','self_escape_max_step_increase_mm','self_escape_complete_at')})
    print('STATIC_BAD',len(bad),json.dumps(bad[:4]))
    print('SCOPE',json.dumps(proof['recorded_interrupted_contact_policy'],ensure_ascii=False))
    frames=FOLDER/'camera/frames.json'
    if frames.exists():
        f=json.loads(frames.read_bytes())['frames'];print('LAST_IMAGE',str(FOLDER/'camera'/f[-1]['image']))
if __name__=='__main__':main()
