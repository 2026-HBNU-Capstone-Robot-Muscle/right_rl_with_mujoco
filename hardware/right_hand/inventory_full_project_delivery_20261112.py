"""Inventory project-owned development material, excluding installed environments.

This writes a local inventory only and never publishes files or opens devices.
Large original recordings remain local and are counted separately.
"""
import json
import os
import shutil
from collections import Counter
from pathlib import Path
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'full_project_delivery_20261112';OUT.mkdir(exist_ok=True)
SKIP_PARTS={'.git','.runtime','.venv','venv','node_modules','site-packages','__pycache__','.aws','.codex','.agents','.ssh','.cache','.pytest_cache','.analysis_packages'}
SKIP_TOP={'deliverables_20261112','full_project_delivery_20261112'}
RAW_EXT={'.avi','.mp4','.mkv','.mov','.m4v','.webm'}
CREDENTIAL_NAMES={'.env','.env.local','.env.production','.netrc','.git-credentials','credentials','credentials.json','token.json','tokens.json','accounts.json','auth.json','hosts.yml','id_rsa','id_ed25519'}
rows=[];excluded=Counter();top=Counter();ext=Counter();large=[]
stack=[ROOT]
while stack:
    folder=stack.pop()
    with os.scandir(folder) as entries:
        for entry in entries:
            path=Path(entry.path);relative=path.relative_to(ROOT)
            if entry.is_symlink():excluded['symlink']+=1;continue
            if entry.is_dir(follow_symlinks=False):
                if entry.name in SKIP_PARTS or (len(relative.parts)==1 and (entry.name in SKIP_TOP or entry.name.startswith('deliverables_'))):
                    excluded['installed_duplicate_or_account_directory']+=1;continue
                if (path/'pyvenv.cfg').is_file():excluded['detected_virtual_environment']+=1;continue
                if relative.parts[:2]==('cloud_handoff_20261106','upload_repo'):
                    excluded['duplicate_personal_upload_snapshot']+=1;continue
                # Camera worker folders contain bulk frame recordings, not source.
                if entry.name=='camera':excluded['raw_camera_directory']+=1;continue
                if relative.as_posix().startswith('team_integration_20261112/right_rl_with_mujoco'):
                    excluded['duplicate_team_checkout']+=1;continue
                stack.append(path);continue
            if not entry.is_file(follow_symlinks=False):continue
            suffix=path.suffix.lower();size=entry.stat(follow_symlinks=False).st_size
            if (entry.name.lower() in CREDENTIAL_NAMES or entry.name.startswith('.env.')
                    or entry.name.endswith(('.pyc','.pyo','.pyd','.dll','.exe','.whl','.so','.pem','.key'))):
                excluded['account_or_installed_binary_file']+=1;continue
            if suffix in RAW_EXT:excluded['raw_video_files']+=1;excluded['raw_video_bytes']+=size;continue
            if (entry.name.endswith('.zip') and len(relative.parts)==1) or suffix=='.bundle':
                excluded['root_archive_duplicates']+=1;continue
            row=dict(path=relative.as_posix(),bytes=size);rows.append(row);top[relative.parts[0]]+=size;ext[suffix]+=size
            if size>=20*1024*1024:large.append(row)
frozen=json.loads((ROOT/'right_hand_rotation_20261106/middle_clearance_probe/review_centered_v2.json').read_bytes())['code_hashes']
selected={r['path'] for r in rows};reference_exceptions=[]
for name in frozen:
    if name in selected:continue
    path=(ROOT/name).resolve();path.relative_to(ROOT)
    if not path.is_file():raise RuntimeError('Missing frozen reference source: '+name)
    size=path.stat().st_size;rows.append(dict(path=name,bytes=size));reference_exceptions.append(name)
    top[Path(name).parts[0]]+=size;ext[path.suffix.lower()]+=size
rows.sort(key=lambda x:x['path']);large.sort(key=lambda x:x['bytes'],reverse=True)
report=dict(kind='project_owned_source_models_learning_results_verification_and_handoff_inventory',
            files=rows,total_files=len(rows),total_bytes=sum(r['bytes'] for r in rows),excluded=dict(excluded),
            largest_files=large,top_directories_by_bytes=top.most_common(25),extensions_by_bytes=ext.most_common(25),
            raw_recordings_remain_local=True,remote_upload_performed=False,
            workspace_free_bytes=shutil.disk_usage(ROOT).free,
            preserves_project_source_directories_named_env=True,
            required_frozen_reference_exceptions=reference_exceptions)
(OUT/'inventory.json').write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k not in ('files','largest_files')},ensure_ascii=False))
print(json.dumps(dict(largest_files=large[:20]),ensure_ascii=False))
