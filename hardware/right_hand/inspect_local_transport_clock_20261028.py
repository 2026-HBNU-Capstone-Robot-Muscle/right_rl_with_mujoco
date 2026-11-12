"""Read local SDK/controller sources and clock metadata; no device instances."""
import inspect,json,time
from pathlib import Path
import dynamixel_sdk as sdk
from leap_calibrated_quarter_20261028 import CalibratedQuarterBus
root=Path(__file__).resolve().parent
print('SDK_PORT_FILE',inspect.getsourcefile(sdk.PortHandler))
for name in ('getCurrentTime','setPacketTimeout','setPacketTimeoutMillis','isPacketTimeout'):
    print(name,inspect.getsource(getattr(sdk.PortHandler,name)))
print('BUS_MRO',[(t.__module__,t.__name__) for t in CalibratedQuarterBus.__mro__])
for base in CalibratedQuarterBus.__mro__:
    for name in ('__init__','_receive'):
        if name in base.__dict__ and base is not object:
            text=inspect.getsource(base.__dict__[name])
            if '_read_recovery_times' in text:print(base.__name__,name,text)
print(json.dumps({n:vars(time.get_clock_info(n)) for n in ('time','monotonic','perf_counter')}))
