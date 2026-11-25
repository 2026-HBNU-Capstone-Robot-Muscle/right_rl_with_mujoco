"""Build a source-preserving cloud subset; no device or network access."""
from pathlib import Path
from collections import deque
import hashlib
import json
import re
import shutil

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
DEST=HERE/'upload_repo'
DENIED={'.runtime','.codex','.agents','.git','.aws','__pycache__','.venv','venv','node_modules','raw'}
DATA_SUFFIXES={'.json','.npz','.mjb','.xml','.stl','.obj','.md','.svg','.html','.txt'}
MODEL_FOLDERS=(
    'right_hand_rotation_20261106/middle_clearance_probe',
    'right_hand_rotation_20261106/taught_upper_clearance',
    'right_hand_rotation_20261106/manual_grasp',
    'right_hand_rotation_20261105/canonical_body_grip/continuous_handoff',
    'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15',
)
SENSITIVE=re.compile(rb'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----|(?:ghp_|github_pat_)[A-Za-z0-9_]{30,}|sk-proj-[A-Za-z0-9_\-]{30,}|AKIA[0-9A-Z]{16}')


def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as source:
        for block in iter(lambda:source.read(1<<20),b''):h.update(block)
    return h.hexdigest()


def main():
    if DEST.exists():raise RuntimeError('upload_repo already exists; preserve it and review before rebuilding')
    ready=json.loads((ROOT/'right_hand_rotation_20261106/middle_clearance_probe/ready.json').read_bytes())
    bindings=ready['code_hashes']
    selected=set();pending=deque();omitted=set()
    def add(path,mandatory=False):
        try:
            path=path.resolve();rel=path.relative_to(ROOT.resolve())
        except (OSError,ValueError):return
        if not path.is_file() or path.is_symlink():return
        if any(part in DENIED or part.startswith('.env') for part in rel.parts) or path.name.lower() in ('auth.json','hosts.yml','credentials','credentials.json'):
            if mandatory:raise RuntimeError(f'Credential/runtime path in required bindings: {rel}')
            return
        if path.stat().st_size>=90_000_000:
            if mandatory:raise RuntimeError(f'Required file exceeds bounded upload size: {rel}')
            omitted.add(rel.as_posix());return
        if rel.as_posix() not in selected:
            selected.add(rel.as_posix())
            if path.suffix.lower()=='.json':pending.append(path)
    for name,expected in bindings.items():
        path=ROOT/name
        if not path.is_file() or sha(path)!=expected:raise RuntimeError(f'Frozen source changed: {name}')
        add(path,True)
    for pattern in ('*.py','*.cmd'):
        for path in ROOT.glob(pattern):add(path)
    for name in ('professor_progress_20261015.md','sequence_flip_poses.json','hand_motor_mapping.py','right_hand_rotation_20261023/native_motor_limits.json','exhibition_20261023/족자_붙여넣기.txt'):
        add(ROOT/name,True)
    for folder in MODEL_FOLDERS:
        for path in (ROOT/folder).rglob('*'):
            if 'raw' not in path.relative_to(ROOT).parts:add(path)
    for folder in ('middle_clearance_probe_20261106_230902_998326','manual_grasp_teach_20261106_183247_578628','natural_cradle_grasp_20261106_175553_695333'):
        for name in ('result.json','cube_grasp_hold.jpg','middle_probe_hold.jpg','capture_01/paired_photo.png'):
            add(ROOT/folder/name)
    for name in ('CLOUD_START_PROMPT.md','웹에서_이어가기.md','last_robot_run_summary.json'):
        add(HERE/name,True)
    for folder in ('right_hand_rotation_20261105/canonical_body_grip','right_hand_rotation_20261030/natural_cradle_grasp'):
        for name in ('offline_summary.json','offline_review.html','selection_review.json','selection_note.md','candidate.npz','proposal.json','request.json','review.json'):
            add(ROOT/folder/name)

    # Follow only existing in-workspace data references. Raw frame collections
    # and arbitrary external files never become upload inputs through strings.
    def follow(text,parent):
        if not isinstance(text,str) or not text or len(text)>1100:return
        candidate=Path(text)
        if candidate.suffix.lower() not in DATA_SUFFIXES:return
        if candidate.is_absolute():add(candidate);return
        for base in (ROOT,parent):
            target=base/candidate
            if target.is_file():add(target);return
    while pending:
        source=pending.popleft()
        try:document=json.loads(source.read_bytes())
        except (ValueError,UnicodeError):continue
        stack=[document]
        while stack:
            value=stack.pop()
            if isinstance(value,dict):
                for key,item in value.items():follow(key,source.parent);stack.append(item)
            elif isinstance(value,list):stack.extend(value)
            elif isinstance(value,str):follow(value,source.parent)

    rows=[]
    for name in sorted(selected):
        source=ROOT/name
        if source.suffix.lower() in {'.py','.cmd','.json','.md','.txt','.html','.xml','.svg'} and SENSITIVE.search(source.read_bytes()):
            raise RuntimeError(f'Possible credential content; review before upload: {name}')
        rows.append({'path':name,'bytes':source.stat().st_size,'sha256':sha(source)})
    DEST.mkdir()
    for row in rows:
        target=DEST/row['path'];target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/row['path'],target)
        if sha(target)!=row['sha256']:raise RuntimeError(f'Copy mismatch: {row["path"]}')
    templates=HERE/'repo_templates'
    for source in templates.rglob('*'):
        if source.is_file():
            target=DEST/source.relative_to(templates);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
    manifest={'kind':'selected_project_snapshot_not_complete_backup','destination':'https://github.com/star1209042/xlerobot.git',
              'branch':'codex/cloud-handoff-20261106','source_files':len(rows),'source_bytes':sum(row['bytes'] for row in rows),
              'frozen_source_bindings':bindings,'source_manifest':rows,'oversized_optional_files_omitted':sorted(omitted),
              'excluded_categories':['installed runtimes','credentials and account sessions','raw camera frames','most historical recordings'],
              'uploaded':False,'hardware_devices_opened':False}
    (DEST/'cloud/upload_manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    (HERE/'upload_selection.json').write_text(json.dumps({k:v for k,v in manifest.items() if k not in ('source_manifest','frozen_source_bindings')},indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    print(json.dumps({'prepared':str(DEST),'files':len(rows),'bytes':manifest['source_bytes'],'bindings_preserved':len(bindings),'credential_pattern_scan':'passed','devices_opened':0},ensure_ascii=False))


if __name__=='__main__':main()
