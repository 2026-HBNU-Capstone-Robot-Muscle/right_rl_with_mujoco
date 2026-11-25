"""Create an immutable no-output pose-feedback variant with explicit sources."""
from pathlib import Path
import json
import hashlib
import shutil

OUT=Path(__file__).resolve().parent
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    root=OUT/'next_cycle/handoff_refinement_20261110/robust_cycle_preparation_20261111';old=root/'shadow_bundle';folder=root/'shadow_bundle_pose_v2';meta=json.loads((old/'bundle.json').read_text(encoding='utf-8'));response=root/'local_pose_response'
    if folder.exists():raise RuntimeError('Preserve identified feedback variant')
    fit=json.loads((response/'review.json').read_text());assert fit['source_guard_thresholds_not_changed'] and sha(response/'response.npz')==fit['response_sha256']
    folder.mkdir()
    for name,digest in meta['files_sha256'].items():
        assert sha(old/name)==digest;target=folder/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(old/name,target)
    shutil.copyfile(response/'response.npz',folder/'response.npz');shutil.copyfile(OUT/'pose_residual_cycle_runtime.py',folder/'pose_residual_cycle_runtime.py')
    meta['files_sha256'].update({name:sha(folder/name) for name in ('response.npz','pose_residual_cycle_runtime.py')});meta.update(format='whole_cycle_pose_residual_no_output_v2',MF_static_margin_mm=.02,phase_speed_mode='fixed2',pose_feedback_model_fit_sha256=sha(response/'review.json'),pose_feedback_is_not_SAC=True,online_simulated_contact_force_not_used=True)
    (folder/'bundle.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(dict(bundle=folder.relative_to(OUT).as_posix(),sha256=sha(folder/'bundle.json'),hardware_execution_allowed=False)),flush=True)


if __name__=='__main__':main()
