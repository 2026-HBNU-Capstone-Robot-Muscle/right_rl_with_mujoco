"""Small scoped command variants; model-only assets, never hardware commands."""
import json,hashlib
from pathlib import Path
from collections import Counter
import numpy as np
from calibrated_quarter_plan_20261028 import LOADED_PATH
from loaded_quarter_reentry_20261025 import limits
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261028/two_axis_connection/actual_handoff_yaw_diagnostic/contact_clearance_family'
def main():
    OUT.mkdir(exist_ok=True);z=np.load(LOADED_PATH);lo,hi=limits();variants=[]
    counts=Counter(str(p) for p in z['phases'])
    for root,thumb in [(-8,-4),(-12,-4),(-8,-8),(-12,-8),(-16,-8)]:
        rows=[];labels=[];last=z['start'].copy();seen=Counter()
        for q,phase in zip(z['goals'],z['phases']):
            phase=str(phase);seen[phase]+=1
            fraction=seen[phase]/counts[phase] if phase=='grasp' else (1-seen[phase]/counts[phase] if phase=='release' else float(phase in ('grasp_hold','yaw','yaw_hold')))
            target=q.copy();target[11]+=int(np.rint(root*fraction));target[0]+=int(np.rint(thumb*fraction))
            assert np.all(target>=lo) and np.all(target<=hi)
            n=max(1,int(np.ceil(max(abs(target-last))/2)))
            rows.extend(np.rint([last+(target-last)*i/n for i in range(1,n+1)]).astype(int));labels.extend([phase]*n);last=target
        rows=np.asarray(rows,dtype=int);assert np.max(abs(np.diff(np.vstack([z['start'],rows]),axis=0)))<=2
        file=OUT/f'ID12_{root:+d}_ID1_{thumb:+d}.npz'
        np.savez_compressed(file,start=z['start'],goals=rows,phases=np.asarray(labels),sample_period_s=.075)
        variants.append(dict(path=file.relative_to(ROOT).as_posix(),offsets_ticks={'12':root,'1':thumb},
            offsets_degrees={'12':root*360/4096,'1':thumb*360/4096},goals=len(rows),native_ranges_preserved=True))
    report=dict(status='model_only_middle_root_thumb_tip_clearance_variants',variants=variants,
        source_path_sha256=hashlib.sha256(LOADED_PATH.read_bytes()).hexdigest(),original_trajectory_changed=False,
        current_gains_limits_or_hardware_changed=False,dynamics_review_required=True,hardware_execution_allowed=False)
    (OUT/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report))
if __name__=='__main__':main()
