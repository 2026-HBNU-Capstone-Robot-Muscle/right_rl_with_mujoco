"""Export current index-first returns at the middleware's existing2tick rate."""
import hashlib
import json
from pathlib import Path
import numpy as np
from rotation_speed_resume_plan_20261125 import ROOT, RotationSpeedTrial

BASE = ROOT/'right_hand_rotation_20261125/middleware_current_rotation'


def main():
    plans = {a:RotationSpeedTrial(a) for a in ('horizontal','vertical')}
    h, v = plans['horizontal'], plans['vertical']
    if not np.array_equal(h.start,v.start):
        raise RuntimeError('Different G endpoints')
    h.rows(h.start)
    v.rows(v.start)
    goals = np.vstack([h.core,v.core])
    if np.max(abs(np.diff(np.vstack([h.start,goals]),axis=0))) > 2:
        raise RuntimeError('Middleware dense export exceeds2ticks')
    phases = np.asarray([*('h_'+str(p) for p in h.core_phases),
                         *('v_'+str(p) for p in v.core_phases)])
    sources = dict(h.manifest['source_hashes'])
    sources[Path(__file__).name] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    BASE.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(BASE/'candidate.npz',start=h.start,goals=goals,phases=phases)
    manifest = dict(kind='current_index_first_dense_middleware_rotations',
        source_hashes=sources, candidate_sha256=hashlib.sha256((BASE/'candidate.npz').read_bytes()).hexdigest(),
        horizontal_goals=len(h.core),vertical_goals=len(v.core),period_seconds=.075,maximum_step_ticks=2,
        source_axis_candidate_sha256={a:p.manifest['candidate_sha256'] for a,p in plans.items()},
        fixed_grip_target_ticks=h.start.tolist(),maximum_extra_READ_pause_s=10.,
        index_then_middle_per_final_stage=True,physical_ROS_execution_verified=False)
    (BASE/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print('Dense middleware export:',len(h.core),len(v.core),'goals; no devices opened.')


if __name__=='__main__':
    main()
