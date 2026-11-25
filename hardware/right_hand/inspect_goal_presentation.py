from pathlib import Path
import importlib.util,json,sys
sys.stdout.reconfigure(encoding='utf-8')
root=Path.cwd()
p=Path.home()/'Documents'/'카카오톡 받은 파일'/'계획발표자료_쾌남근육맨_최종본_v2.pdf'
print('PDF',p,p.exists())
for mod in ('fitz','pypdf','pypdfium2','PIL','cv2'):
    print('MODULE',mod,bool(importlib.util.find_spec(mod)))
for parent in (root,*root.parents):
    f=parent/'AGENTS.md'
    if f.exists():print('INSTRUCTIONS',f,f.read_text(encoding='utf-8'))
out=root/'goal_presentation_review';out.mkdir(exist_ok=True)
if importlib.util.find_spec('fitz'):
    import fitz
    doc=fitz.open(p);parts=[]
    for i,page in enumerate(doc):
        text=page.get_text();parts.append(f'\nPAGE {i+1}\n{text}')
        page.get_pixmap(matrix=fitz.Matrix(1,1)).save(str(out/f'page_{i+1:02d}.png'))
    (out/'extracted.txt').write_text(''.join(parts),encoding='utf-8')
    print(''.join(parts))
elif importlib.util.find_spec('pypdf'):
    from pypdf import PdfReader
    parts=[f'\nPAGE {i+1}\n{page.extract_text()}' for i,page in enumerate(PdfReader(p).pages)]
    (out/'extracted.txt').write_text(''.join(parts),encoding='utf-8');print(''.join(parts))
for path in root.rglob('*'):
    if path.suffix.lower() in ('.mp4','.gif','.webm') and not any(x in path.parts for x in ('.runtime','.git')):
        if 'sac' in str(path.relative_to(root)).lower():print('MEDIA',path.relative_to(root),path.stat().st_size)
