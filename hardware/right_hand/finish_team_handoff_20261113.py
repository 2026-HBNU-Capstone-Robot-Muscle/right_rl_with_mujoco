"""Preview and package only the portable handoff, not the full source archive."""
import hashlib
import json
import os
import re
import subprocess
import zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'team_handoff_20261113'
browser=Path(os.environ.get('PROGRAMFILES(X86)','C:/Program Files (x86)'))/'Microsoft/Edge/Application/msedge.exe'
if browser.is_file():
    env=os.environ.copy();env.pop('ELECTRON_RUN_AS_NODE',None)
    document=(OUT/'PROFESSOR_MEETING.html').read_text(encoding='utf-8')
    sections=re.findall(r'<section class="page" id="page\d+">.*?</section>',document,re.S)
    assert len(sections)==2
    for page in (1,2):
        image=OUT/f'.preview_page{page}.png'
        preview=OUT/f'.preview_document_page{page}.html'
        preview.write_text(document.split('<body>',1)[0]+'<body>'+sections[page-1]+'</body></html>',encoding='utf-8')
        r=subprocess.run([str(browser),'--headless=new','--disable-gpu','--disable-background-networking','--no-first-run',
            '--user-data-dir='+str(OUT/'.render_profile'),'--window-size=1100,1300','--force-device-scale-factor=1',
            '--virtual-time-budget=1500','--screenshot='+str(image),preview.as_uri()],
            capture_output=True,env=env,timeout=45,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        if r.returncode or not image.exists():raise RuntimeError('Preview failed')
pdf=OUT/'PROFESSOR_MEETING.pdf'
assert len(re.findall(rb'/Type\s*/Page\b',pdf.read_bytes()))==2
for name in ['START_HERE.md','PROFESSOR_MEETING.md']:
    text=(OUT/name).read_text(encoding='utf-8')
    headings=re.findall(r'^## (.+)$',text,re.M)
    assert headings[-1].startswith('마지막:'),'Keep unfinished work in final section'
selected=[p for p in OUT.rglob('*') if p.is_file() and not any(part.startswith('.') for part in p.relative_to(OUT).parts)
          and p.suffix.lower()!='.zip' and p.name not in ['PACKAGE_MANIFEST.json','DELIVERY.json']]
manifest={p.relative_to(OUT).as_posix():dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in selected}
meta=OUT/'PACKAGE_MANIFEST.json';meta.write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
selected.append(meta)
archive=OUT/'TEAM_HANDOFF_20261113.zip'
with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    for p in sorted(selected):z.write(p,p.relative_to(ROOT).as_posix())
with zipfile.ZipFile(archive) as z:assert z.testzip() is None
receipt=dict(kind='documentation_and_latest_analysis_supplement',full_source_archive=False,archive=archive.name,
    files=len(selected),bytes=archive.stat().st_size,sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
    pdf_pages=2,unfinished_items_in_last_section=True,includes_installed_environments=False,
    existing_code_branch='feature/hardware',existing_code_commit='76a2438147207978ecf972d54a0904edb1180ea6',
    startup_exception_runtime_patch_applied=json.loads((OUT/'evidence/SOURCES.json').read_bytes())['startup_exception_runtime_patch_applied'],new_github_upload_performed=False,new_hardware_access=False)
(OUT/'DELIVERY.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(receipt,ensure_ascii=False),flush=True)
