"""Preserve source model and create a numerically converged CCD variant."""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,save_json,sha


def main():
    folder=OUT/'next_cycle/handoff_refinement_20261110/collision_convergence';folder.mkdir(exist_ok=True)
    source,_,_,_,provenance=assets('active');variant,_,_,_,_=assets('active')
    old=int(source.opt.ccd_iterations);variant.opt.ccd_iterations=50
    unchanged=[];changed=[]
    for name in dir(source):
        if name.startswith('_'):continue
        try:a=getattr(source,name);b=getattr(variant,name)
        except Exception:continue
        if isinstance(a,np.ndarray):
            (unchanged if np.array_equal(a,b) else changed).append(name)
    if changed:raise AssertionError(('Unexpected model array changes',changed))
    options=[]
    for name in dir(source.opt):
        if name.startswith('_'):continue
        a=getattr(source.opt,name);b=getattr(variant.opt,name)
        if isinstance(a,(int,float,np.ndarray)) and not np.array_equal(a,b):options.append(name)
    if options!=['ccd_iterations']:raise AssertionError(('Unexpected numerical changes',options))
    path=folder/'model_ccd50.mjb';mujoco.mj_saveModel(variant,str(path))
    readback=mujoco.MjModel.from_binary_path('variant.mjb',assets={'variant.mjb':path.read_bytes()})
    assert int(readback.opt.ccd_iterations)==50
    for name in unchanged:
        if not np.array_equal(getattr(source,name),getattr(readback,name)):raise AssertionError(('Readback model data changed',name))
    report=dict(status='isolated_CCD_iteration_variant',source_provenance=provenance,source_model_sha256=provenance['model_sha256'],
                variant_model_path=str(path.relative_to(OUT)),variant_model_sha256=sha(path),
                changed_options=dict(ccd_iterations=[old,50]),unchanged_model_array_fields=unchanged,
                all_model_arrays_unchanged=True,native_ccd_algorithm_and_shapes_and_limits_unchanged=True,
                source_file_preserved=True,previous_policy_passes_do_not_transfer=True,
                devices_opened=0,hardware_execution_allowed=False)
    save_json(folder/'manifest.json',report)
    base=OUT/'next_cycle/staged_shadow_bundle_v2';bundle=OUT/'next_cycle/staged_shadow_bundle_v2_ccd50';bundle.mkdir(exist_ok=True)
    meta=json.loads((base/'bundle.json').read_text())
    for name in meta['files_sha256']:(bundle/name).write_bytes((base/name).read_bytes())
    meta.update(model_sha256=sha(path),model_provenance=dict(model_kind='active_ccd50',model_path=str(path.relative_to(OUT)),model_sha256=sha(path)),
                numerical_collision_iterations=50,source_bundle_sha256=sha(base/'bundle.json'),
                connected_model_cases={},first_turn_model_nominal_validated=False,
                evidence='Research variant: previous10-iteration model validations do not transfer; fresh full evaluation is required',
                physical_camera_registration_verified=False,hardware_execution_allowed=False)
    (bundle/'bundle.json').write_text(json.dumps(meta,indent=2),encoding='utf-8')
    print(json.dumps(dict(unchanged_model_arrays=len(unchanged),changed_options=report['changed_options'],variant_sha256=sha(path),bundle=str(bundle))),flush=True)


if __name__=='__main__':main()
