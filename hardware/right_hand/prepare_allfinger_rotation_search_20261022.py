"""Distinct offline search allowing ring extension, never further palm curl."""
from pathlib import Path
root=Path(__file__).resolve().parent
source=(root/'search_cube_rotation_extended_20261022.py').read_text(encoding='utf-8')
changes=[
    ('VARIABLES=np.arange(12)  # All ring goals remain fixed.','VARIABLES=np.arange(16)  # Ring tip/PIP/root may extend; no deeper palm curl.'),
    ('ends[:,[7,11]]=np.maximum(ends[:,[7,11]],base[[7,11]])',
     'ends[:,[7,11]]=np.maximum(ends[:,[7,11]],base[[7,11]])\n        ends[:,[12,14,15]]=np.minimum(ends[:,[12,14,15]],base[[12,14,15]])'),
    ("mean=np.asarray(prior['validation'][0]['delta']).ravel();std[:]=28.",
     "prior_delta=np.asarray(prior['validation'][0]['delta']).reshape(2,-1)\n        padded=np.zeros((2,len(VARIABLES)));padded[:,:prior_delta.shape[1]]=prior_delta\n        mean=padded.ravel();std[:]=28."),
    ("if best is not None and iteration==0:\n                for index,scale", "if best is not None and iteration==0:\n                for index,scale"),
]
for old,new in changes:
    assert source.count(old)==1,old
    source=source.replace(old,new)
old="            ranked=sorted(pool.map(score,population),key=lambda r:r['cost'])"
new="""            if best is not None and iteration==0:
                for index,travel in enumerate((64,128,192,256),7):
                    if index>=len(population):break
                    trial=np.asarray(best['delta']).reshape(2,16).copy()
                    trial[:,[12,14,15]]-=np.array([[travel//2],[travel]])
                    population[index]=trial.ravel()
            ranked=sorted(pool.map(score,population),key=lambda r:r['cost'])"""
assert source.count(old)==1
source=source.replace(old,new)
path=root/'search_cube_rotation_allfinger_20261022.py'
path.write_text(source,encoding='utf-8');compile(source,str(path),'exec')
print(path.name)
