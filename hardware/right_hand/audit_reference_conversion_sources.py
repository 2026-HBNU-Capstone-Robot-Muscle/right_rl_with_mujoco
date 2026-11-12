"""Source/readback audit only; no hardware modules instantiated."""
import ast,hashlib,json
from pathlib import Path
root=Path(__file__).resolve().parent
paths=[root.parent/'work/Leap_Test/deploy/leap_hw.py',
       Path('C:/Users/김준서/Documents/GitHub/LEAP_Hand_API/python/leap_hand_utils/leap_hand_utils.py')]
report={'hardware_commands_sent':0,'sources':[]}
for p in paths:
    if not p.exists():
        report['sources'].append({'path':str(p),'missing':True});continue
    text=p.read_text(encoding='utf-8-sig');tree=ast.parse(text)
    snippets=[]
    for node in ast.walk(tree):
        if isinstance(node,(ast.FunctionDef,ast.Assign)):
            section=ast.get_source_segment(text,node)
            if any(x in section for x in ('OFFSET','SIM2ARR','sim_to_hw','hw_to_sim','allegro_to_leap','sim_ones_to_leap','sim_to_real','leap_to_sim','np.pi','3.14159')):
                snippets.append(section)
    report['sources'].append(dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),snippets=snippets))
data=json.loads((root/'pose_pair_readonly_20261018_032605_137975/result.json').read_text(encoding='utf-8'))
report['middle_saved_settings']=[{k:m[k] for k in ('id','drive_mode','homing_offset_raw','position_ticks_signed')} for m in data['snapshots'][0]['motors'] if 9<=m['id']<=12]
(root/'middle_side_review_20261018/conversion_sources.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(report,ensure_ascii=True,indent=2))
