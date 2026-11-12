"""Inspect a bounded upload selection and package metadata, without devices."""
from pathlib import Path
import importlib.metadata as md
import importlib.util
import json

ROOT = Path(__file__).resolve().parent.parent
OUT = Path(__file__).resolve().parent


def main():
    ready = json.loads((ROOT / 'right_hand_rotation_20261106/middle_clearance_probe/ready.json').read_bytes())
    names = set(ready['code_hashes'])
    for folder in (
        'right_hand_rotation_20261106/middle_clearance_probe',
        'right_hand_rotation_20261106/taught_upper_clearance',
        'right_hand_rotation_20261105/canonical_body_grip/continuous_handoff',
        'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15',
    ):
        for path in (ROOT / folder).rglob('*'):
            if path.is_file() and not path.is_symlink() and 'raw' not in path.relative_to(ROOT).parts:
                names.add(path.relative_to(ROOT).as_posix())
    rows=[]
    for name in sorted(names):
        path=ROOT/name
        if path.is_file():rows.append({'path':name,'bytes':path.stat().st_size,'required_binding':name in ready['code_hashes']})
    payload={'files':len(rows),'bytes':sum(r['bytes'] for r in rows),'largest':sorted(rows,key=lambda r:r['bytes'],reverse=True)[:16],
             'root_code':{'files':len(list(ROOT.glob('*.py'))),'bytes':sum(p.stat().st_size for p in ROOT.glob('*.py'))},
             'packages':{}}
    for name in ('numpy','scipy','mujoco','jax','opencv-python','opencv-contrib-python','mujoco-playground','dynamixel-sdk','pyserial'):
        try:payload['packages'][name]=md.version(name)
        except md.PackageNotFoundError:pass
    spec=importlib.util.find_spec('mujoco_playground')
    if spec:payload['mujoco_playground_origin']=spec.origin
    (OUT/('upload_inspection_linux.json' if __import__('os').name=='posix' else 'upload_inspection_windows.json')).write_text(json.dumps(payload,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    print(json.dumps(payload,ensure_ascii=False))


if __name__=='__main__':main()
