"""Exact single-forward finite empty-hand path review, no device access."""
import hashlib
import screen_pitch_handoff_loaded_20261028 as s
from single_forward_plan_20261030 import OUT,HAND_PATH
def main():
 before=hashlib.sha256(HAND_PATH.read_bytes()).hexdigest();s.OUT=OUT;s.LOADED_PATH=HAND_PATH;s.main()
 if before!=hashlib.sha256(HAND_PATH.read_bytes()).hexdigest():raise RuntimeError('Single-forward path changed during review')
if __name__=='__main__':main()
