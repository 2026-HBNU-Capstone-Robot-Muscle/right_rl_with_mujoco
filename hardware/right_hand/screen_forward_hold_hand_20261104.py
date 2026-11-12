import os
os.environ['JAX_PLATFORMS']='cpu'
from pathlib import Path
import screen_pitch_handoff_loaded_20261028 as screen
R=Path(__file__).resolve().parent
if __name__=='__main__':
    screen.OUT=R/'right_hand_rotation_20261104/forward_hold';screen.LOADED_PATH=screen.OUT/'candidate.npz';screen.main()
