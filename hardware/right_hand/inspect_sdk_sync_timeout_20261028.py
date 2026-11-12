"""Inspect installed SDK methods without opening any devices."""
import inspect
from dynamixel_sdk.protocol2_packet_handler import Protocol2PacketHandler
print(inspect.getsource(Protocol2PacketHandler.readRx))
print(inspect.getsource(Protocol2PacketHandler.syncReadTx))
