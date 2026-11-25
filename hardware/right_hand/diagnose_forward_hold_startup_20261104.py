"""Print a delayed Python stack if model imports are unexpectedly slow."""
import faulthandler,time
faulthandler.dump_traceback_later(20,repeat=False)
began=time.perf_counter()
print('Importing simulator modules for startup diagnosis only.',flush=True)
import screen_yaw_roundtrip_model_20261030
faulthandler.cancel_dump_traceback_later()
print('Simulator import completed in',round(time.perf_counter()-began,2),'seconds; no model run or devices opened.',flush=True)
