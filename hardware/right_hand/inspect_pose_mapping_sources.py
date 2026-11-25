"""Local source search for encoder-to-simulator contracts; no device access."""
from pathlib import Path
ROOT=Path(__file__).resolve().parent
terms=('sim_order','joint_order','motor_order','motor_ids','joint_names','np.pi','math.pi','offset','sign')
for p in ROOT.glob('*.py'):
    if not any(s in p.name for s in ('bridge','deploy','hardware','readonly','contract','adapter','mapping','export','stage0')):continue
    lines=p.read_text(encoding='utf-8',errors='replace').splitlines()
    matches=[(i,l) for i,l in enumerate(lines,1) if any(t in l.lower() for t in terms)]
    if matches:
        print('\nFILE',p.name)
        for i,l in matches[:16]:print(i,l[:220])
print('\nNearby source directories')
for root in (ROOT.parent/'work',Path(r'C:\Users\김준서\Documents\GitHub\LEAP_Hand_API\python')):
    if root.exists(): print(root,[p.name for p in root.iterdir()][:50])
for p in (ROOT.parent/'work/Leap_Test/deploy/leap_hw.py',
          Path(r'C:\Users\김준서\Documents\GitHub\LEAP_Hand_API\python\leap_hand_utils\leap_hand_utils.py')):
    print('\nSOURCE',p)
    if p.exists(): print(p.read_text(encoding='utf-8',errors='replace')[:18000])
