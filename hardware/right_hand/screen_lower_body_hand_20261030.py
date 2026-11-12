"""Whole new hand path, cube disabled; existing finite dynamics bounds."""
from pathlib import Path
import screen_pitch_handoff_loaded_20261028 as s
R=Path(__file__).resolve().parent
if __name__=='__main__':
 s.OUT=R/'right_hand_rotation_20261030/lower_body_regrasp';s.LOADED_PATH=s.OUT/'candidate.npz';s.main()
