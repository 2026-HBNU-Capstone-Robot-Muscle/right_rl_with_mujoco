"""Find original recorded motor evidence without reading user documents."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def main():
    selected=[]
    for path in ROOT.glob('*.json'):
        if any(s in path.name for s in ('sequence','anchor_capture_result','support_capture_result')):
            if path.stat().st_size>150_000_000:continue
            try:r=json.loads(path.read_bytes())
            except (ValueError,OSError):continue
            if not isinstance(r,dict):continue
            if isinstance(r.get('samples'),list) and r['samples']:selected.append(dict(file=path.name,bytes=path.stat().st_size,status=r.get('status'),keys=list(r),samples=len(r['samples'])))
    print(json.dumps(selected,ensure_ascii=False,indent=2))
    folders=[]
    for p in sorted(ROOT.glob('sequence_vision_202610*')):
        if '20261010' in p.name or '20261011' in p.name:folders.append(dict(folder=p.name,files=[v.name for v in p.glob('*.json')]))
    print('EARLY FULL RUN FOLDERS',json.dumps(folders,ensure_ascii=False))
if __name__=='__main__':main()
