from pathlib import Path
import zipfile,xml.etree.ElementTree as ET,re,json,sys
sys.stdout.reconfigure(encoding='utf-8')
p=Path.home()/'Documents'/'카카오톡 받은 파일'/'캡스톤2 중간발표자료_v2.pptx'
out=Path('goal_presentation_review');ns={'a':'http://schemas.openxmlformats.org/drawingml/2006/main'}
pages=[]
with zipfile.ZipFile(p) as z:
    names=sorted((n for n in z.namelist() if re.fullmatch(r'ppt/slides/slide\d+\.xml',n)),key=lambda n:int(re.search(r'(\d+)\.xml',n)[1]))
    for name in names:
        root=ET.fromstring(z.read(name));lines=[''.join(n.itertext()) for n in root.findall('.//a:t',ns)]
        text=' | '.join(lines);pages.append({'page':len(pages)+1,'text':text})
        print('SLIDE',len(pages),text)
(out/'latest_ppt_text.json').write_text(json.dumps(pages,ensure_ascii=False,indent=2),encoding='utf-8')
d=json.loads(Path('leap_id12_backoff_empty_result.json').read_text())
print('RESULT',{k:v for k,v in d.items() if k in ('timestamp','status','approach_hold','backoff','return_hold','all_torque_off_verified','settings_restored')})
for phase in ('id12_approach_hold','id12_backoff','id12_return_hold'):
    rows=[r for r in d['samples'] if r['phase']==phase]
    print('PHASE',phase,'rows',len(rows))
    for r in ([rows[0],rows[len(rows)//2],rows[-1]] if rows else []):
        print({k:r.get(k) for k in ('t','actual','goal','current_ma','present_pwm_raw','present_velocity_rpm')})
