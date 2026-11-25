"""Create a distinct offline search version; preserve the completed small search."""
from pathlib import Path

root = Path(__file__).resolve().parent
source = (root / 'search_first_cube_rotation_20261022.py').read_text(encoding='utf-8')
changes = [
    ("parser.add_argument('--resume');parser.add_argument('--all-resets',action='store_true')",
     "parser.add_argument('--resume');parser.add_argument('--all-resets',action='store_true')\n    parser.add_argument('--endpoint-limit-ticks',type=int,default=192)"),
    ("OUT.mkdir(exist_ok=True);began=time.monotonic()",
     "OUT.mkdir(exist_ok=True);began=time.monotonic()\n    if not 96<=args.endpoint_limit_ticks<=256:raise ValueError('Offline endpoint bound')\n    amplitude=args.endpoint_limit_ticks;first_limit=int(amplitude*2/3);leg_steps=int(np.ceil(amplitude/2))"),
    ("delta[0]=np.clip(delta[0],-64,64)", "delta[0]=np.clip(delta[0],-first_limit,first_limit)"),
    ("delta[1]=np.clip(delta[1],-96,96)", "delta[1]=np.clip(delta[1],-amplitude,amplitude)"),
    ("delta[1]=np.minimum(np.maximum(delta[1],delta[0]-80),delta[0]+80)",
     "delta[1]=np.minimum(np.maximum(delta[1],delta[0]-amplitude),delta[0]+amplitude)"),
    ("last+(end-last)*k/40).astype(int) for k in range(1,41)",
     "last+(end-last)*k/leg_steps).astype(int) for k in range(1,leg_steps+1)"),
    ("elif self_peak>1e-6:fault='new_self_contact'",
     "elif any(n in touched for n in ('support_glass','support_wood')):fault='cube_external_support'\n            elif self_peak>1e-6:fault='new_self_contact'"),
    ("if k>=80 and qualifies else 0", "if k>=len(goals) and qualifies else 0"),
    ("or k==99", "or k==len(goals)+19"),
    ("std[:]=20.", "std[:]=28."),
    ("goal_period_s=.05,motion_samples=80,hold_samples=20,",
     "goal_period_s=.05,motion_samples=len(path(best['delta'])),hold_samples=20,\n        endpoint_limit_ticks=amplitude,script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),"),
]
for old, new in changes:
    if source.count(old) != 1:
        raise RuntimeError('Search template changed: '+old)
    source = source.replace(old, new)
path = root / 'search_cube_rotation_extended_20261022.py'
path.write_text(source, encoding='utf-8')
compile(source, str(path), 'exec')
print('Offline search prepared:', path.name)
