from pathlib import Path
import screen_pitch_handoff_loaded_20261028 as s
R=Path(__file__).resolve().parent
if __name__=='__main__':s.OUT=R/'right_hand_rotation_20261104/raised_cradle_160_yaw0';s.LOADED_PATH=s.OUT/'candidate.npz';s.main()
