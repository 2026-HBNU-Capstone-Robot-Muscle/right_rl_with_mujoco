"""Extract actual perturbed checkpoints, retaining all saved dynamic state."""
from pathlib import Path
import json
import numpy as np
from native_cycle import OUT,assets,BOUND,save_json,sha


def main():
    root=OUT/'next_cycle/handoff_refinement_20261110/robust_cycle_preparation_20261111';_,initial,_,_,_=assets('active_ccd50');rows=[]
    for case in ('initial_cube_worldX_plus1mm','sliding_mu0p25'):
        source=root/'failure_checkpoints'/case/'state_pool.npz';pool=np.load(source,allow_pickle=False);index=int(np.flatnonzero(pool['indices']==1700)[0]);target=source.parent/'state1700.npz'
        if target.exists():raise RuntimeError('Preserve actual checkpoint')
        np.savez_compressed(target,**{k:pool[k][index] for k in BOUND},time=pool['time'][index],up=initial['up'],initial_cube_rotation=initial['initial_cube_rotation'],initial_cube_center=initial['initial_cube_center'])
        rows.append(dict(case=case,source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),state=target.relative_to(OUT).as_posix(),state_sha256=sha(target),after_goals=1700,friction_hypothesis=.25 if case=='sliding_mu0p25' else None))
    save_json(root/'teacher_checkpoints.json',dict(status='actual_initial_prefix1700_extracted',rows=rows,not_a_cold_placed_grip=True,devices_opened=0,hardware_execution_allowed=False));print(json.dumps(rows),flush=True)


if __name__=='__main__':main()
