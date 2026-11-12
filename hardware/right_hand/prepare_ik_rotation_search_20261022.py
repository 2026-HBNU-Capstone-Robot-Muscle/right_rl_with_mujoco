"""Create an offline IK-seeded trajectory search variant."""
from pathlib import Path
root=Path(__file__).resolve().parent
source=(root/'search_cube_rotation_extended_20261022.py').read_text(encoding='utf-8')
old='    with ThreadPoolExecutor(max_workers=args.workers) as pool:\n'
new='''    ik=json.loads((OUT/'rigid_yaw_ik_seeds.json').read_bytes())
    if ik['source_sha256']!=hashlib.sha256(SOURCE.read_bytes()).hexdigest():raise ValueError('IK source changed')
    for seed in ik['seeds']:
        trial=score(np.asarray(seed['delta']).ravel())
        print('IK_SEED',json.dumps({k:v for k,v in trial.items() if k not in ('trials','delta','endpoint_ticks')}),flush=True)
        if best is None or trial['cost']<best['cost']:best=trial
    if best is not None:mean=np.asarray(best['delta']).ravel()
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
'''
assert source.count(old)==1
source=source.replace(old,new)
path=root/'search_cube_rotation_ik_20261022.py'
path.write_text(source,encoding='utf-8')
compile(source,str(path),'exec')
print(path.name)
