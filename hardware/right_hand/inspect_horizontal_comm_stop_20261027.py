"""Saved failure and transport sources only; no serial/camera access."""
import inspect,json
from pathlib import Path
from leap_horizontal_quarter_20261027 import HorizontalQuarterBus
ROOT=Path(__file__).resolve().parent
r=json.loads((ROOT/'horizontal_quarter_trial_20261027_211439_985362/result.json').read_text(encoding='utf-8'))
print(json.dumps(dict(enabled_at=r.get('enabled_at'),read_failures=r.get('read_failures'),
    final_check_error=r.get('final_check_error'),last_issued_phase=r.get('last_issued_phase'),
    last_issued_command=r.get('last_issued_command'),write_events=r.get('write_events',[])[-5:]),ensure_ascii=False))
print('METHOD OWNERS')
for method in ('_sync_read','write','flags','block','torque_off'):
    owners=[c for c in HorizontalQuarterBus.__mro__ if method in c.__dict__]
    for owner in owners:
        value=owner.__dict__[method]
        print(method,owner.__module__,inspect.getsourcelines(value)[1])
    if owners and method in ('_sync_read','flags'):
        print(inspect.getsource(owners[0].__dict__[method]))
    elif owners and method=='write':print(inspect.getsource(owners[-1].__dict__[method]))
