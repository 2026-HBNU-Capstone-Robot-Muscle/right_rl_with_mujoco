"""List unique generated snapshot support sources without reading accounts."""
import json
import os
from pathlib import Path
ROOT=Path(__file__).resolve().parent
snapshot=ROOT/'cloud_handoff_20261106/upload_repo';rows=[]
for folder,dirs,files in os.walk(snapshot):
    dirs[:]=[n for n in dirs if n not in ('.git','.runtime','.aws','.codex','.agents','__pycache__')]
    for name in files:
        path=Path(folder)/name;relative=path.relative_to(snapshot)
        if (ROOT/relative).exists():continue
        if path.suffix not in ('.py','.md','.txt','.json','.sh') and name not in ('.gitattributes','.gitignore'):continue
        rows.append(dict(path=relative.as_posix(),bytes=path.stat().st_size))
print(json.dumps(rows,ensure_ascii=False))
(ROOT/'full_project_delivery_20261112/snapshot_only_sources.json').write_text(json.dumps(rows,indent=2,ensure_ascii=False),encoding='utf-8')
