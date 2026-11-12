"""Capture real device-free unittest output for the final reviewed code."""
import subprocess,sys
from pathlib import Path
root=Path(__file__).resolve().parent
r=subprocess.run([sys.executable,'-X','utf8','-B','-m','unittest','test_goal_aware_progress_guard_20261025','test_reverse_quarter_resume_empty_20261025'],cwd=root,capture_output=True,text=True,encoding='utf-8',timeout=180)
output=r.stdout+r.stderr
(root/'right_hand_rotation_20261024/reverse_quarter_resume_empty/fixture_v4/test_completion.log').write_text(output,encoding='utf-8')
print(output,end='');raise SystemExit(r.returncode)
