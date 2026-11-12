"""Version source-only updates of a no-output research bundle; never deploys."""
from pathlib import Path
import json
import hashlib
import shutil

OUT=Path(__file__).resolve().parent
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    folder=OUT/'next_cycle/handoff_refinement_20261110/robust_cycle_preparation_20261111/shadow_bundle';meta_path=folder/'bundle.json';meta=json.loads(meta_path.read_text(encoding='utf-8'))
    old=sha(meta_path);archive=folder.parent/'bundle_source_revisions'/old;archive.mkdir(parents=True,exist_ok=True);shutil.copyfile(meta_path,archive/'bundle.json')
    for name in ('whole_cycle_shadow_runtime.py','cycle_joint_clearance.py','shadow_runtime.py','staged_shadow_runtime.py'):
        shutil.copyfile(folder/name,archive/name);shutil.copyfile(OUT/name,folder/name);meta['files_sha256'][name]=sha(folder/name)
    assert not meta['hardware_execution_allowed'] and meta['accepted_registration_id'] is None
    meta_path.write_text(json.dumps(meta,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(dict(previous_metadata_sha256=old,current_metadata_sha256=sha(meta_path),hardware_execution_allowed=False)),flush=True)


if __name__=='__main__':main()
