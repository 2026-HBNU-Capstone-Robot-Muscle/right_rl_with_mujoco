from pathlib import Path
import re,zlib,sys,json
sys.stdout.reconfigure(encoding='utf-8')
p=Path.home()/'Documents'/'카카오톡 받은 파일'/'계획발표자료_쾌남근육맨_최종본_v2.pdf'
b=p.read_bytes();print('BYTES',len(b));print(b[:100])
out=Path('goal_presentation_review');out.mkdir(exist_ok=True)
objects={}
for m in re.finditer(rb'(\d+)\s+(\d+)\s+obj\b(.*?)endobj',b,re.S):
    ident=int(m[1]);body=m[3];objects[ident]=body
    if b'stream' in body:
        h,data=body.split(b'stream',1);data=data.lstrip(b'\r\n').rsplit(b'endstream',1)[0].rstrip(b'\r\n')
        if b'/FlateDecode' in h:
            try:data=zlib.decompress(data)
            except Exception:continue
        if b'beginbf' in data or b'BT' in data or b'/ObjStm' in h:
            (out/f'object_{ident}.bin').write_bytes(data)
            print('STREAM',ident,h[:200],data[:600])
    elif any(x in body for x in (b'/Type /Page',b'/Type/Page',b'/ToUnicode',b'/Font')):
        print('OBJECT',ident,body[:700])
for name in ('supported_batch_eval.json','settings.json','result.json'):
    d=json.loads((Path('sac_forward_support_reference_v1_20k')/name).read_text())
    print('SIM',name,{k:v for k,v in d.items() if k not in ('results','trace','metrics','config','summaries')})
    if 'summaries' in d:print('SIM_SUMMARIES',d['summaries'][:2])
