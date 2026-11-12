"""Reuse the exact saved contact inspector without importing robot workflows."""
import ast
import hashlib
from pathlib import Path
import numpy as np

SOURCE=Path(__file__).resolve().parent/'screen_loaded_repeat_20261026.py'
_raw=SOURCE.read_bytes()
SOURCE_SHA256=hashlib.sha256(_raw).hexdigest()
_tree=ast.parse(_raw)
_node=next(n for n in _tree.body if isinstance(n,ast.FunctionDef) and n.name=='cached_contacts')
_namespace={'np':np}
exec(compile(ast.Module(body=[_node],type_ignores=[]),str(SOURCE),'exec'),_namespace)
cached_contacts=_namespace['cached_contacts']
