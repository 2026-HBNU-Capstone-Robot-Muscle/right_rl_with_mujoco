"""Local text extraction for this unencrypted PowerPoint PDF using its ToUnicode maps."""
from pathlib import Path
import re,zlib,sys,json
sys.stdout.reconfigure(encoding='utf-8')
source=Path.home()/'Documents'/'카카오톡 받은 파일'/'계획발표자료_쾌남근육맨_최종본_v2.pdf'
objs={int(m[1]):m[2] for m in re.finditer(rb'(\d+)\s+0\s+obj\b(.*?)endobj',source.read_bytes(),re.S)}
def stream(body):
    h,s=body.split(b'stream',1);s=s.lstrip(b'\r\n').rsplit(b'endstream',1)[0].rstrip(b'\r\n')
    return zlib.decompress(s) if b'/FlateDecode' in h else s
maps={}
for obj,body in objs.items():
    match=re.search(rb'/ToUnicode\s+(\d+)\s+0\s+R',body)
    if not match:continue
    data=stream(objs[int(match[1])]);mapping={}
    for section in re.findall(rb'beginbfchar(.*?)endbfchar',data,re.S):
        for a,b in re.findall(rb'<([0-9A-Fa-f]+)>\s*<([0-9A-Fa-f]+)>',section):
            mapping[int(a,16)]=bytes.fromhex(b.decode()).decode('utf-16be')
    for section in re.findall(rb'beginbfrange(.*?)endbfrange',data,re.S):
        for m in re.finditer(rb'<([0-9A-Fa-f]+)>\s*<([0-9A-Fa-f]+)>\s*(<([0-9A-Fa-f]+)>|\[(.*?)\])',section,re.S):
            a,b=int(m[1],16),int(m[2],16)
            if m[4]:
                start=int(m[4],16);width=len(m[4])//2
                for i in range(b-a+1):mapping[a+i]=(start+i).to_bytes(width,'big').decode('utf-16be')
            else:
                for i,v in enumerate(re.findall(rb'<([0-9A-Fa-f]+)>',m[5])):mapping[a+i]=bytes.fromhex(v.decode()).decode('utf-16be')
    maps[obj]=mapping
pages=[int(v) for v in re.findall(rb'(\d+)\s+0\s+R',re.search(rb'/Kids\[(.*?)\]',objs[2],re.S)[1])]
allpages=[]
for pageid in pages:
    body=objs[pageid];fontblock=re.search(rb'/Font\s*<<(.*?)>>',body,re.S)[1]
    fonts={a.decode():int(b) for a,b in re.findall(rb'/(F\d+)\s+(\d+)\s+0\s+R',fontblock)}
    content=stream(objs[int(re.search(rb'/Contents\s+(\d+)\s+0\s+R',body)[1])]);spans=[]
    for bt in re.findall(rb'\bBT\b(.*?)\bET\b',content,re.S):
        f=re.search(rb'/(F\d+)\s+[\d.]+\s+Tf',bt)
        if not f:continue
        fontid=fonts[f[1].decode()];cmap=maps.get(fontid);parts=[]
        tm=re.search(rb'([\d.-]+)\s+([\d.-]+)\s+Tm',bt)
        # Only strings in text-showing arrays, not language/metadata dictionaries.
        for shown in re.findall(rb'\[(.*?)\]\s*TJ',bt,re.S):
            for token in re.finditer(rb'<([0-9A-Fa-f\s]+)>|\(((?:\\.|[^\\)])*)\)',shown,re.S):
                if token[1]:raw=bytes.fromhex(token[1].decode())
                else:
                    raw=re.sub(rb'\\([()\\])',rb'\1',token[2])
                if cmap:
                    parts.append(''.join(cmap.get(int.from_bytes(raw[i:i+2],'big'),'�') for i in range(0,len(raw),2)))
                else:parts.append(raw.decode('cp1252',errors='replace'))
        if parts:spans.append(dict(x=float(tm[1]) if tm else 0,y=float(tm[2]) if tm else 0,text=''.join(parts)))
    lines=[]
    for span in sorted(spans,key=lambda s:(-round(s['y']/2),s['x'])):
        if lines and abs(lines[-1]['y']-span['y'])<2:lines[-1]['text']+=' '+span['text']
        else:lines.append(dict(y=span['y'],text=span['text']))
    allpages.append(dict(page=len(allpages)+1,text='\n'.join(line['text'] for line in lines)))
out=Path('goal_presentation_review');out.mkdir(exist_ok=True)
(out/'pages_text.json').write_text(json.dumps(allpages,ensure_ascii=False,indent=2),encoding='utf-8')
text='\n\n'.join(f'PAGE {p["page"]}\n{p["text"]}' for p in allpages)
(out/'extracted.txt').write_text(text,encoding='utf-8');print(text)
