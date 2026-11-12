"""Freeze the retained-contact hypothesis for separate full-route reviews."""
import json,hashlib
from pathlib import Path
import numpy as np
from gapped_inverse_plan_20261029 import extra_rows
from new_pair_grasp_plan_20261027 import OPEN_PATH
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261029/contact_retained_inverse'
SOURCE=ROOT/'right_hand_rotation_20261029/inverse_contact_search/candidate_011.npz'
def main():
    OUT.mkdir(exist_ok=True);destination=OUT/'candidate.npz'
    if destination.exists():raise RuntimeError('Refuse to overwrite frozen candidate')
    destination.write_bytes(SOURCE.read_bytes());data=np.load(destination)
    added=extra_rows();rows=added+[(str(p),q.copy()) for p,q in zip(data['phases'],data['goals'])]
    opened=np.load(OPEN_PATH)['open_goal']
    np.savez_compressed(OUT/'whole_hand_path.npz',start=opened,goals=np.asarray([q for _,q in rows]),phases=np.asarray([p for p,_ in rows]))
    report=dict(candidate=destination.relative_to(ROOT).as_posix(),candidate_sha256=hashlib.sha256(destination.read_bytes()).hexdigest(),
        source=SOURCE.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        whole_hand_goals=len(rows),loaded_goals=len(data['goals']),current_scope='model_research_only',
        physical_inverse_90_verified=False,hardware_execution_allowed=False,devices_opened=False)
    (OUT/'preparation.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report))
if __name__=='__main__':main()
