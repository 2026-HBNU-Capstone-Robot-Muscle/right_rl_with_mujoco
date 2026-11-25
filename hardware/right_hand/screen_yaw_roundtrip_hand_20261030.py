"""Finite hand dynamics of the continuous composition. Cube disabled."""
import hashlib
import screen_pitch_handoff_loaded_20261028 as s
from yaw_roundtrip_plan_20261030 import OUT,HAND_PATH
def main():
    before=hashlib.sha256(HAND_PATH.read_bytes()).hexdigest();s.OUT=OUT;s.LOADED_PATH=HAND_PATH;s.main()
    if before!=hashlib.sha256(HAND_PATH.read_bytes()).hexdigest():raise RuntimeError('Composed path changed during hand review')
if __name__=='__main__':main()
