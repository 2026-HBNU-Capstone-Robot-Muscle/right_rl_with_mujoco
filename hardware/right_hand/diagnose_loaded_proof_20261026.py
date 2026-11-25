"""Read-only in-memory instrumentation of a failed saved-proof comparison."""
import ast,copy,json
from loaded_thumb_rotation_plan_20261026 import ROOT,FIXTURE,LoadedThumbRotationPlan,extra_hashes
def main():
    path=ROOT/'loaded_thumb_rotation_gate_20261026.py';tree=ast.parse(path.read_text(encoding='utf-8'))
    for node in ast.walk(tree):
        if isinstance(node,ast.If) and isinstance(node.test,ast.BoolOp) and isinstance(node.test.op,ast.Or):
            if not any(isinstance(v,ast.Constant) and isinstance(v.value,str) and v.value.startswith('Book grasp release/path/code/checkpoint') for child in node.body for v in ast.walk(child)):continue
            expressions=node.test.values
            issues=ast.List(elts=[ast.IfExp(test=copy.deepcopy(v),body=ast.Constant(ast.unparse(v)),orelse=ast.Constant(None)) for v in expressions],ctx=ast.Load())
            node.body=ast.parse("raise RuntimeError('Failed fields: '+repr([]))").body
            node.body[0].exc.args[0].right.args[0]=issues
    ast.fix_missing_locations(tree);ns={'__file__':str(path),'__name__':'_loaded_proof_diagnostic'}
    exec(compile(tree,str(path),'exec'),ns)
    proof=json.loads((FIXTURE/'screen.json').read_bytes());saved=proof['extra_code_hashes'];current=extra_hashes()
    print('EXTRA_HASH_KEYS',dict(only_saved=sorted(saved.keys()-current.keys()),only_current=sorted(current.keys()-saved.keys()),
        changed=[k for k in saved.keys()&current.keys() if saved[k]!=current[k]]))
    ns['validate_proof'](LoadedThumbRotationPlan(FIXTURE/'request.json'),FIXTURE/'screen.json')
if __name__=='__main__':main()
