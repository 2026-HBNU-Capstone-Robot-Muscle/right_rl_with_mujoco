import hashlib,json
from pathlib import Path
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261105/preintegration/post_off_cleanup_fix/source_before'
def main():
    O.mkdir(parents=True,exist_ok=True);proof=R/'right_hand_rotation_20261104/forward_hold/review.json';review=json.loads(proof.read_bytes());files=('forward_hold_plan_20261104.py','forward_hold_gate_20261104.py','leap_forward_hold_observe_20261104.py');out={}
    for name in files:
        data=(R/name).read_bytes();h=hashlib.sha256(data).hexdigest()
        if review['code_hashes'][name]!=h:raise RuntimeError('Source not equal to admitted proof')
        dest=O/name
        if dest.exists() and dest.read_bytes()!=data:raise RuntimeError('Archive differs')
        dest.write_bytes(data);out[name]=h
    dest=O/'review.json'
    if dest.exists() and dest.read_bytes()!=proof.read_bytes():raise RuntimeError('Proof archive differs')
    dest.write_bytes(proof.read_bytes());out['review.json']=hashlib.sha256(proof.read_bytes()).hexdigest()
    (O/'manifest.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8');print(json.dumps(out))
if __name__=='__main__':main()
