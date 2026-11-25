"""Create run-specific analysis/export copies, leaving previous evidence intact."""
from pathlib import Path
ROOT=Path(__file__).resolve().parent
old='adaptive_yaw_loaded_20261029_160451_416563';new='gapped_inverse_trial_20261029_184252_272166'
for source,target in (('check_observed638_palm_20261029.py','check_gapped_inverse_palm_20261029.py'),
    ('export_adaptive_loaded_video_20261029.py','export_gapped_inverse_video_20261029.py')):
    text=(ROOT/source).read_text(encoding='utf-8')
    assert text.count(old)==1
    (ROOT/target).write_text(text.replace(old,new),encoding='utf-8')
    print(target)
