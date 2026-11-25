from pathlib import Path
import calibrated_quarter_plan_20261028 as p
source=p.ROOT/'right_hand_rotation_20261028/reconnected_170143_margin_v4/request.json'
out=p.ROOT/'right_hand_rotation_20261028/reconnected_170143_margin_v4_checked';out.mkdir(exist_ok=True)
target=out/'request.json'
if target.exists():raise RuntimeError('Refuse to overwrite checked request')
target.write_bytes(source.read_bytes())
print(target.relative_to(p.ROOT).as_posix())
