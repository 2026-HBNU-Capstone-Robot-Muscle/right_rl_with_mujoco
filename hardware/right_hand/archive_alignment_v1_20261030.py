import shutil
from pathlib import Path
R=Path(__file__).resolve().parent;out=R/'right_hand_rotation_20261030/placement_alignment_capture/v1_sources';out.mkdir(exist_ok=True)
for n in ('placement_alignment_plan_20261030.py','placement_alignment_gate_20261030.py','placement_alignment_capture_20261030.py','placement_alignment_camera_20261030.py','screen_placement_alignment_20261030.py','leap_placement_alignment_20261030.py','leap_placement_alignment.cmd','test_placement_alignment_20261030.py'):
 p=out/n
 if p.exists():raise RuntimeError('Refuse to overwrite archived v1 source')
 shutil.copyfile(R/n,p)
print('Archived the exact earlier photo-only sources.')
