"""Inspect saved result structure and available offline runtimes only."""
from pathlib import Path
import json,importlib.util,sys

ROOT=Path(__file__).resolve().parent
RUN=ROOT/'middle_clearance_probe_20261106_230902_998326'


def main():
    result=json.loads((RUN/'result.json').read_bytes())
    fields={key:({'type':'dict','keys':list(value)[:25]} if isinstance(value,dict) else
                 {'type':'list','length':len(value),'first_keys':list(value[0]) if value and isinstance(value[0],dict) else None}
                 if isinstance(value,list) else value) for key,value in result.items()}
    modules={name:bool(importlib.util.find_spec(name)) for name in ('numpy','cv2','mujoco','scipy','pip')}
    optional=ROOT/'.analysis_packages'
    installed=[p.name for p in optional.iterdir() if p.is_dir()] if optional.exists() else []
    print(json.dumps({'python':sys.version,'fields':fields,'modules':modules,'analysis_packages':installed[:35]},ensure_ascii=False))
    for key in ('samples','log','rows','motion_log','health_log'):
        rows=result.get(key)
        if isinstance(rows,list) and rows:
            print(json.dumps({'list':key,'first':rows[0],'last':rows[-1]},ensure_ascii=False)[:9000])


if __name__=='__main__':main()
