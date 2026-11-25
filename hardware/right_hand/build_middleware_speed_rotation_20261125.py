"""Export the exact current selected speed paths and their dense source proof."""
import hashlib
import json
from pathlib import Path
import numpy as np
from rotation_speed_resume_plan_20261125 import ROOT, RotationSpeedTrial, SLOW_SEATING_PHASES

BASE=ROOT/'right_hand_rotation_20261125/middleware_speed_rotation'


def main():
    paths={}
    for axis in ('horizontal','vertical'):
        plan=RotationSpeedTrial(axis)
        selected=plan.rows(plan.start)
        mask=np.asarray([p.startswith('trial_'+axis+'_') for p in plan.labels])
        offset=len(plan.original_rows)-len(plan.core)-1
        indices=plan.indices[mask]-offset
        goals=selected[mask]
        phases=np.asarray([p[len('trial_'+axis+'_'):] for p in np.asarray(plan.labels)[mask]])
        limits=np.asarray([2 if p in SLOW_SEATING_PHASES else 6 for p in np.asarray(plan.labels)[mask]])
        if (indices[0]!=0 or indices[-1]!=len(plan.core)-1
                or not np.array_equal(goals,plan.core[indices])
                or np.any(np.max(abs(np.diff(np.vstack([plan.start,goals]),axis=0)),axis=1)>limits)):
            raise RuntimeError('Selected speed path proof mismatch: '+axis)
        paths[axis]=(plan,goals,phases,indices,limits)
    h,hg,hp,hi,hl=paths['horizontal']
    v,vg,vp,vi,vl=paths['vertical']
    if not np.array_equal(h.start,v.start):raise RuntimeError('Different G')
    BASE.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(BASE/'candidate.npz',start=h.start,goals=np.vstack([hg,vg]),
        phases=np.asarray([*('h_'+str(p) for p in hp),*('v_'+str(p) for p in vp)]),
        dense_goals=np.vstack([h.core,v.core]),
        dense_phases=np.asarray([*('h_'+str(p) for p in h.core_phases),*('v_'+str(p) for p in v.core_phases)]),
        source_indices=np.r_[hi,len(h.core)+vi],step_limits=np.r_[hl,vl])
    sources=dict(h.manifest['source_hashes'])
    sources[Path(__file__).name]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    manifest=dict(kind='current_source_selected_speed_rotations_with_dense_proof',source_hashes=sources,
        candidate_sha256=hashlib.sha256((BASE/'candidate.npz').read_bytes()).hexdigest(),
        horizontal_goals=len(hg),vertical_goals=len(vg),
        horizontal_dense_goals=len(h.core),vertical_dense_goals=len(v.core),
        period_seconds=.075,maximum_step_ticks=6,entry_maximum_step_ticks=2,
        dense_proof_maximum_step_ticks=2,maximum_extra_READ_pause_s=10.,
        source_axis_candidate_sha256={a:p[0].manifest['candidate_sha256'] for a,p in paths.items()},
        index_then_middle_per_final_stage=True,physical_ROS_execution_verified=False)
    (BASE/'manifest.json').write_bytes((json.dumps(manifest,ensure_ascii=False,indent=2)+'\n').encode('utf-8'))
    print('Selected middleware speed export:',len(hg),len(vg),'goals; original dense2tick proof retained.')


if __name__=='__main__':main()
