"""Additional ring separation for a research candidate; no device access."""
import json,hashlib
from pathlib import Path
import numpy as np
from loaded_quarter_reentry_20261025 import limits
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261029/inverse_wide_ring'
def interp(a,b):
    n=max(1,int(np.ceil(np.max(abs(b-a))/2)))
    return np.rint([a+(b-a)*i/n for i in range(1,n+1)]).astype(int)
def main():
    OUT.mkdir(exist_ok=True);source=ROOT/'right_hand_rotation_20261029/inverse_contact_search_v2/candidate_056.npz'
    z=np.load(source);reports=[]
    frontmask=np.isin(z['phases'],['stage_5_loaded_grasp','stage_6_grasp_hold'])
    yaw=z['goals'][z['phases']=='stage_7_horizontal_yaw'][64:]
    hold=z['goals'][z['phases']=='stage_8_horizontal_hold']
    release=z['goals'][z['phases']=='stage_9_loaded_open'][:-64]
    last=z['goals'][-1].copy();ready=z['goals'][frontmask][-1].copy()
    assert len(yaw)>250 and np.all(yaw[:,13]==2432) and np.all(release[:,13]==2432)
    lo,hi=limits()
    for clearance in (192,256):
        name=f'ring{clearance}';folder=OUT/name;folder.mkdir(exist_ok=True)
        cleared=ready.copy();cleared[13]=2560-clearance
        turn=np.vstack([interp(ready,cleared),yaw.copy()]);turn[:,13]=np.r_[interp(ready,cleared)[:,13],np.repeat(2560-clearance,len(yaw))]
        held=hold.copy();held[:,13]=2560-clearance
        opened=last.copy();opened[13]=2560-clearance
        rel=release.copy();rel[:,13]=2560-clearance;rel=np.vstack([rel,interp(opened,last)])
        goals=np.vstack([z['goals'][frontmask],turn,held,rel,np.repeat(last[None,:],20,axis=0)])
        phases=np.r_[z['phases'][frontmask],['stage_7_horizontal_yaw']*len(turn),['stage_8_horizontal_hold']*len(held),['stage_9_loaded_open']*len(rel),['stage_10_remove_hold']*20]
        values=np.vstack([z['start'],goals]);valid=bool(np.all(values>=lo) and np.all(values<=hi) and np.max(abs(np.diff(values,axis=0)))<=2)
        if not valid:raise RuntimeError('Existing native/step bound failed')
        target=folder/'candidate.npz';np.savez_compressed(target,start=z['start'],goals=goals,phases=phases,empty_preposition=z['empty_preposition'],empty_gap_open=z['empty_gap_open'])
        row=dict(name=name,source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
            candidate=target.relative_to(ROOT).as_posix(),candidate_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),loaded_goals=len(goals),
            native_and_step_bounds=valid,finite_physics_verified=False,hardware_execution_allowed=False,devices_opened=False)
        reports.append(row);print(json.dumps(row))
    (OUT/'build_review.json').write_text(json.dumps(dict(cases=reports),indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
