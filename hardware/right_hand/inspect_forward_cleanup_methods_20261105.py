import inspect
from leap_forward_hold_observe_20261104 import ForwardHoldBus
for name in ('write','off','_receive'):
    f=getattr(ForwardHoldBus,name)
    print(name,inspect.getsourcefile(f))
    print(inspect.getsource(f))
