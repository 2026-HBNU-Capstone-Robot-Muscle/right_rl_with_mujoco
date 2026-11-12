"""Keep the rejected coarse grid and join all27 refined dynamic cases."""
import hashlib,json
import numpy as np
from adaptive_yaw_loaded_plan_20261029 import ROOT,OUT,STRESS_PATH,hashes
reports=[];sources=[];canonical=None
for front in (-15,-10,-5):
    folder=OUT/f'loaded_pause_1ms_front{front}';path=folder/'review.json';r=json.loads(path.read_bytes())
    assert r['simulation_timestep_s']==.001 and r['settling_seconds']==2 and r['physics_acceptance_bounds_unchanged']
    assert r['candidate_sha256']==hashlib.sha256(STRESS_PATH.read_bytes()).hexdigest() and len(r['cases'])==9
    assert {(v['book_front_mm'],v['gain_scale'],v['friction']) for v in r['cases']}=={(float(front),g,mu) for g in (.5,1.,1.5) for mu in (.2,.3,.5)}
    assert json.loads((folder/'control_source_hashes.json').read_bytes())==hashes()
    z=np.load(folder/'loaded_path.npz');arrays={name:z[name] for name in z.files}
    if canonical is None:canonical=arrays
    else:
        assert set(arrays)==set(canonical)
        for key in arrays:np.testing.assert_array_equal(arrays[key],canonical[key])
    sources.append(dict(path=path.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),all_passed=r['all_passed']))
    reports.extend(r['cases'])
folder=OUT/'loaded_pause_refined_27';folder.mkdir(exist_ok=True)
np.savez_compressed(folder/'loaded_path.npz',**canonical)
r.update(cases=reports,all_passed=all(v['passed'] for v in reports),blocks=sources,
    loaded_path_sha256=hashlib.sha256((folder/'loaded_path.npz').read_bytes()).hexdigest(),
    coarse_rejected_review=(OUT/'loaded_pause_stress_27/review.json').relative_to(ROOT).as_posix())
(folder/'review.json').write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(cases=len(reports),all_passed=r['all_passed'],simulation_timestep_s=.001)))
