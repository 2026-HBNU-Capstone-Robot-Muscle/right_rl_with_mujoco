import json
import inspect
from pathlib import Path
import sequence_flip_checked as checked
import dynamixel_sdk as sdk

for p in [Path.cwd(),*Path.cwd().parents]:
    a=p/'AGENTS.md'
    if a.exists(): print('AGENTS',a,a.read_text(encoding='utf-8'))
for folder in sorted(Path.cwd().glob('bus_echo_off_20261023_*'))[-2:]:
    r=json.loads((folder/'result.json').read_text(encoding='utf-8'))
    rows=r.get('reads',[])
    print(folder.name,{k:r.get(k) for k in ('status','error','failed_phase','all_torque_off_verified','goal_restoration_verified')})
    print('reads',len(rows),'goal_writes',r.get('goal_writes',[]),'phases',r.get('phases',[]))
    print('goal_equals_position',[x['goal']==x['position'] for x in rows])
    if rows: print('first',rows[0],'last',rows[-1])
print('MRO',checked.Bus.__mro__)
print('BLOCK',inspect.getsource(checked.Bus.block))
print('BLOCK ONCE',inspect.getsource(checked.Bus._block_once))
print('GETDATA',inspect.getsource(sdk.GroupSyncRead.getData))
print('CMD',Path('leap_bus_echo_off.cmd').read_text(encoding='utf-8'))
print('README',Path('leap_bus_echo_off_README.md').read_text(encoding='utf-8'))
print('STATUS',Path('sac_current_status.md').read_text(encoding='utf-8')[:5000])
