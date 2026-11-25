"""Refresh the selected update inventory without changing frozen motion evidence."""
import hashlib
import json
import shutil
from pathlib import Path
from prepare_github_hardware_update_20261112 import ROOT,PRIVATE,WORK

def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    path=PRIVATE/'cloud/hardware_update_manifest_20261112.json';update=json.loads(path.read_bytes());files=update['files']
    selected=set(files)
    for pattern in ('*tip_first*20261112.py','*lower_grip_actual*20261112.py'):
        selected.update(p.relative_to(ROOT).as_posix() for p in ROOT.glob(pattern))
    for folder in ('tip_first_trial','tip_first_trial_v2','tip_first_transfer_v1','tip_first_transfer_v2','tip_first_transfer_v3','lower_grip_actual_review'):
        for p in (ROOT/'right_hand_rotation_20261112'/folder).rglob('*'):
            if p.is_file() and (p.suffix in ('.json','.npz','.md','.py') or (folder=='lower_grip_actual_review' and p.suffix=='.jpg')):
                selected.add(p.relative_to(ROOT).as_posix())
    for name in ('leap_cycle_tip_first_transfer.cmd','right_hand_rotation_20261112/support_first_transfer/FINAL_TARGET.json','team_integration_20261112/README.md',
                 'team_integration_20261112/team_push_receipt.json','cloud_handoff_20261106/CLOUD_START_PROMPT.md','professor_progress_20261015.md'):
        if (ROOT/name).is_file():selected.add(name)
    for name in sorted(selected):
        source=ROOT/name
        if not source.is_file():continue
        target=PRIVATE/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(source,target);files[name]=digest(target)
    update.update(files=files,full_rotation_cycle_complete=False,raw_camera_recording_directory_included=False,
        selected_review_images=sum(name.startswith('right_hand_rotation_20261112/lower_grip_actual_review/') and name.endswith('.jpg') for name in files))
    path.write_text(json.dumps(update,indent=2,ensure_ascii=False),encoding='utf-8')
    inventory_path=PRIVATE/'cloud/upload_manifest.json';inventory=json.loads(inventory_path.read_bytes());changes=[]
    frozen=inventory['frozen_source_bindings']
    for name,expected in frozen.items():
        if digest(PRIVATE/name)!=expected:raise RuntimeError('Frozen motion source changed: '+name)
    for entry in inventory['source_manifest']:
        actual=digest(PRIVATE/entry['path'])
        if actual!=entry['sha256']:
            if entry['path'] in frozen:raise RuntimeError('Do not refresh a frozen source digest')
            changes.append(dict(path=entry['path'],previous_sha256=entry['sha256'],new_sha256=actual))
            entry['sha256']=actual
            if 'bytes' in entry:entry['bytes']=(PRIVATE/entry['path']).stat().st_size
            if 'size' in entry:entry['size']=(PRIVATE/entry['path']).stat().st_size
    previous_changes=inventory.get('hardware_update_20261112',{}).get('original_snapshot_inventory_revisions',[])
    inventory['hardware_update_20261112']=dict(extra_inventory='cloud/hardware_update_manifest_20261112.json',branch='feature/hardware',
        original_snapshot_inventory_revisions=previous_changes+changes,frozen_motion_evidence_rewritten=False)
    inventory_path.write_text(json.dumps(inventory,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps(dict(update_files=len(files),original_inventory_entries_refreshed=len(changes),frozen_bindings_verified=len(frozen),raw_frames_copied=0)))

if __name__=='__main__':main()
