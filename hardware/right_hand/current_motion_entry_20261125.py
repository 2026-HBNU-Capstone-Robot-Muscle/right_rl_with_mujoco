"""Portable entry for the checked current horizontal/vertical/keyboard trial."""
import argparse
import hashlib
import importlib
import importlib.util
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parent
COMPAT={
    'leap_hw':'bcac41f9ab95a406c3ad571a84c7bdb4a98ff44f420759f204b266b627c452eb',
    'limits':'4852b466d90023519fc941622ae31e20457bad8518dbb201abb934232d685299',
}


def prepare_imports():
    sys.path.insert(0,str(ROOT))
    for name,digest in COMPAT.items():
        path=ROOT/'current_compat'/(name+'.py')
        if hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
            raise RuntimeError('Historical import helper changed: '+name)
        loaded=sys.modules.get(name)
        if loaded is not None:
            if hashlib.sha256(Path(loaded.__file__).read_bytes()).hexdigest()!=digest:
                raise RuntimeError('Conflicting historical module: '+name)
            continue
        spec=importlib.util.spec_from_file_location(name,path)
        module=importlib.util.module_from_spec(spec)
        sys.modules[name]=module
        try:spec.loader.exec_module(module)
        except BaseException:
            sys.modules.pop(name,None)
            raise


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode',choices=['horizontal','vertical','keyboard'],default='keyboard')
    p.add_argument('--execute',action='store_true')
    args=p.parse_args(argv)
    prepare_imports()
    if args.mode=='keyboard':
        module=importlib.import_module('leap_keyboard_rotate_grip_20261125')
        command=[]
    else:
        module=importlib.import_module('leap_rotation_speed_resume_20261125')
        command=['--axis',args.mode]
    if args.execute:command.append('--execute')
    return module.main(command)


if __name__=='__main__':raise SystemExit(main())
