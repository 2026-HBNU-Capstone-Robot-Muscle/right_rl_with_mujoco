"""Existing local files/libraries only; no install, WSL execution or devices."""
import importlib.util,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
roots=[ROOT/'right_hand_rotation_20261027',ROOT/'right_hand_rotation_20261026',ROOT.parent/'work/Leap_Test',ROOT.parent/'work',ROOT/'.runtime']
hits=[]
for folder in roots:
    if folder.exists():
        for pattern in ('*distal*.stl','*distal*.obj','*distal*.npz','*distal*.ply'):
            for path in folder.rglob(pattern):
                text=str(path)
                if text not in hits:hits.append(text)
report=dict(assets=hits,libraries={n:importlib.util.find_spec(n) is not None for n in ('numpy','trimesh','scipy','fcl','mujoco')})
print(json.dumps(report,ensure_ascii=False))
(ROOT/'right_hand_rotation_20261027/power_branch_20261027/local_cad_inventory.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
