"""Print only bus read/sample definitions needed for the stored timeout."""
import ast
import inspect
from pathlib import Path

ROOT = Path(__file__).resolve().parent
for filename in ('sequence_flip_checked.py', 'hand_prepare_full.py',
                 'hand_prepare_resume.py'):
    if not (ROOT / filename).is_file():
        continue
    source = (ROOT / filename).read_text(encoding='utf-8')
    tree = ast.parse(source)
    for node in tree.body:
        if isinstance(node, (ast.Import, ast.ImportFrom, ast.ClassDef)):
            print(ast.get_source_segment(source, node).splitlines()[0])
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name in (
                'block', 'snapshot', 'sample', 'preflight', '_read_sample'):
            print(filename, node.name, node.lineno)
            print(ast.get_source_segment(source, node))

import sequence_flip_checked
for cls in sequence_flip_checked.Bus.__mro__:
    print('MRO', cls.__module__, cls.__name__)
    for name in ('block', '_block_once', '_receive'):
        if name in cls.__dict__:
            print(inspect.getsource(cls.__dict__[name]))
