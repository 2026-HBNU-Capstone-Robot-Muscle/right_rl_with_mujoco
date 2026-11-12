"""One insertion; the original motion runner remains untouched."""
from pathlib import Path
ROOT=Path(__file__).resolve().parent
old=(ROOT/'horizontal_quarter_motion_runner_20261027.py').read_text(encoding='utf-8')
pad=' '*12
needle=pad+'state,errors,cycle_start=observe(phase)\n'+pad+'bus.write(116,4,target,verify=False)'
addition='\n'.join(pad+line for line in (
    'state,errors,cycle_start=observe(phase)',
    "lag_hook=getattr(bus,'root_lag_pause',None)",
    'if lag_hook is not None:',
    '    state,errors,cycle_start=lag_hook.wait_if_needed(phase,state,errors,cycle_start,',
    '        observe=lambda:observe(phase),command=command,report=report,sleep=sleep,cycle_now=now)',
    'bus.write(116,4,target,verify=False)'))
if old.count(needle)!=1:raise RuntimeError('Original runner insertion point differs')
(ROOT/'root_lag_motion_runner_20261029.py').write_text(old.replace(needle,addition),encoding='utf-8')
print('Created separate runner; original checks and code unchanged except the single wait hook.')
