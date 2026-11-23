import inspect
import dynamixel_sdk
for cls in (dynamixel_sdk.GroupSyncRead,):
    for name in ('rxPacket','txRxPacket','getError'):
        if hasattr(cls,name):print(inspect.getsource(getattr(cls,name)))
