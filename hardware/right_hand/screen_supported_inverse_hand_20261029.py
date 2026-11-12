"""Whole route finite EMPTY checks; inherited collision bounds unchanged."""
import hashlib
import screen_pitch_handoff_loaded_20261028 as screen
from supported_inverse_empty_plan_20261029 import OUT,HAND_PATH
def main():
    before=hashlib.sha256(HAND_PATH.read_bytes()).hexdigest();screen.OUT=OUT;screen.LOADED_PATH=HAND_PATH;screen.main()
    if before!=hashlib.sha256(HAND_PATH.read_bytes()).hexdigest():raise RuntimeError('Hand route changed during dynamics review')
if __name__=='__main__':main()
