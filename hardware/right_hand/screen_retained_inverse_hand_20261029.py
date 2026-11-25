"""Device-free finite EMPTY review of the newly frozen complete hand route."""
from pathlib import Path
import screen_pitch_handoff_loaded_20261028 as screen
ROOT=Path(__file__).resolve().parent
screen.OUT=ROOT/'right_hand_rotation_20261029/contact_retained_inverse'
screen.LOADED_PATH=screen.OUT/'whole_hand_path.npz'
if __name__=='__main__':screen.main()
