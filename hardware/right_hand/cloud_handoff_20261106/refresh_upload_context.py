"""Refresh only handoff documents in a prepared snapshot and its manifest."""
from pathlib import Path
import hashlib,json,shutil

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
DEST=HERE/'upload_repo'
names=('cloud_handoff_20261106/CLOUD_START_PROMPT.md','cloud_handoff_20261106/웹에서_이어가기.md','professor_progress_20261015.md')
manifest=json.loads((DEST/'cloud/upload_manifest.json').read_bytes())
for name in names:
    target=DEST/name;shutil.copy2(ROOT/name,target)
    row=next(row for row in manifest['source_manifest'] if row['path']==name)
    row.update(bytes=target.stat().st_size,sha256=hashlib.sha256(target.read_bytes()).hexdigest())
for source in (HERE/'repo_templates').rglob('*'):
    if source.is_file():
        target=DEST/source.relative_to(HERE/'repo_templates');target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
manifest['source_bytes']=sum(row['bytes'] for row in manifest['source_manifest'])
manifest.pop('uploaded',None)
manifest['remote_upload_status_at_snapshot_creation']='not_uploaded'
(DEST/'cloud/upload_manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
print(json.dumps({'context_refreshed':True,'source_files':len(manifest['source_manifest']),'source_bytes':manifest['source_bytes']}))
