"""Small placement perturbations, explicitly distinct from 27 gain/material cases."""
import sys,json
from pathlib import Path
import screen_lower_body_free_20261030 as s
R=Path(__file__).resolve().parent
if __name__=='__main__':
 for offset in ((5,0,0),(-5,0,0),(0,5,0),(0,-5,0)):
  name='offset_'+'_'.join(str(x) for x in offset);folder='right_hand_rotation_20261030/lower_body_regrasp/'+name
  sys.argv=['offsets','--candidate','right_hand_rotation_20261030/lower_body_regrasp/candidate.npz','--output-folder',folder,'--nominal-only','--front','-10','--seat-up','--offset-mm',*[str(x) for x in offset]];s.main();r=json.loads((R/folder/'review.json').read_bytes());c=r['cases'][0];print('PLACEMENT OFFSET',json.dumps(dict(offset_mm=offset,safety_passed=c['passed'],forward_deg=c['forward_from_placement_deg'],body_hold=c['body_hold_observation'])),flush=True)
