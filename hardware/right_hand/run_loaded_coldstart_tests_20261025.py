import subprocess,sys
from pathlib import Path
root=Path(__file__).resolve().parent
r=subprocess.run([sys.executable,'-X','utf8','-B','-m','unittest','test_loaded_quarter_trial_20261025'],cwd=root,capture_output=True,text=True,encoding='utf-8',timeout=180)
output=r.stdout+r.stderr
(root/'right_hand_rotation_20261024/loaded_quarter_trial_20261025/fixture_v6/test_completion.log').write_text(output,encoding='utf-8')
print(output,end='');raise SystemExit(r.returncode)
