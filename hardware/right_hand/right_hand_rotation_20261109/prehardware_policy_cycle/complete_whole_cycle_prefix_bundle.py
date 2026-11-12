"""Retain every original SAC bundle verification input in the new package."""
from pathlib import Path
import json
import hashlib
import shutil

OUT=Path(__file__).resolve().parent
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    root=OUT/'next_cycle/handoff_refinement_20261110/robust_cycle_preparation_20261111';bundle=root/'shadow_bundle';old=OUT/'next_cycle/staged_shadow_bundle_v2_ccd50';meta=json.loads((bundle/'bundle.json').read_text(encoding='utf-8'));original=json.loads((old/'bundle.json').read_text(encoding='utf-8'))
    before=sha(bundle/'bundle.json');archive=root/'bundle_source_revisions'/before;archive.mkdir(parents=True,exist_ok=True);shutil.copyfile(bundle/'bundle.json',archive/'bundle.json')
    for name in ('bundle.json',*original['files_sha256']):
        source=old/name;target=bundle/'sac_prefix'/name;target.parent.mkdir(parents=True,exist_ok=True)
        if name!='bundle.json':assert sha(source)==original['files_sha256'][name]
        shutil.copyfile(source,target);meta['files_sha256'][target.relative_to(bundle).as_posix()]=sha(target)
    (bundle/'bundle.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2),encoding='utf-8')
    (root/'encoder_pose_feedback_v1/startup_error.json').write_text(json.dumps(dict(status='startup_failed_before_any_model_step',reason='Original SAC bundle references source files that were absent in the first package',model_steps=0,devices_opened=0,hardware_execution_allowed=False),indent=2),encoding='utf-8')
    print(json.dumps(dict(prefix_files_verified=len(original['files_sha256']),bundle_metadata_sha256=sha(bundle/'bundle.json'),devices_opened=0)),flush=True)


if __name__=='__main__':main()
