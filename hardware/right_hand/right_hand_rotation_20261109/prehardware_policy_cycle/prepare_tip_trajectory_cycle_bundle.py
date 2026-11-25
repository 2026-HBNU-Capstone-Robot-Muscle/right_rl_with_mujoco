"""Prepare one explicit no-output tip-trajectory comparison variant."""
from pathlib import Path
import json
import hashlib
import shutil

OUT=Path(__file__).resolve().parent
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    root=OUT/'next_cycle/handoff_refinement_20261110/robust_cycle_preparation_20261111';old=root/'shadow_bundle';folder=root/'shadow_bundle_tip_trajectory_v3';meta=json.loads((old/'bundle.json').read_text(encoding='utf-8'))
    if folder.exists():raise RuntimeError('Preserve tip-trajectory comparison variant')
    folder.mkdir()
    for name,digest in meta['files_sha256'].items():
        assert sha(old/name)==digest;target=folder/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(old/name,target)
    name='contact_trajectory_cycle_runtime.py';shutil.copyfile(OUT/name,folder/name);meta['files_sha256'][name]=sha(folder/name);meta.update(format='whole_cycle_tip_trajectory_no_output_v3',MF_static_margin_mm=.04,phase_speed_mode='fixed2',clearance_selection_method='soft_tip_trajectory_cost_not_contact_invariant')
    (folder/'bundle.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(dict(bundle=folder.relative_to(OUT).as_posix(),hardware_execution_allowed=False)),flush=True)


if __name__=='__main__':main()
