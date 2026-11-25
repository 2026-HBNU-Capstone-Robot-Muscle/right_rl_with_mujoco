"""Build a portable handoff and printable meeting brief from existing evidence."""
import hashlib
import html
import importlib.util
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path
from urllib.parse import quote

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'team_handoff_20261113'
COMMIT='76a2438147207978ecf972d54a0904edb1180ea6'
REMOTE='https://github.com/2026-HBNU-Capstone-Robot-Muscle/right_rl_with_mujoco/blob/'+COMMIT+'/hardware/right_hand/'
CHECKOUT=ROOT/'github_desktop/right_rl_with_mujoco/hardware/right_hand'

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def copy(relative,destination):
    source=ROOT/relative;target=OUT/destination
    target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(source,target)
    return dict(source=relative,packaged_as=destination,sha256=sha(source))

def inline(text):
    text=html.escape(text)
    text=re.sub(r'!\[([^\]]*)\]\(([^)]+)\)',r'<figure><img alt="\1" src="\2"><figcaption>\1</figcaption></figure>',text)
    text=re.sub(r'\[([^\]]+)\]\(([^)]+)\)',r'<a href="\2">\1</a>',text)
    text=re.sub(r'\*\*([^*]+)\*\*',r'<strong>\1</strong>',text)
    text=re.sub(r'`([^`]+)`',r'<code>\1</code>',text)
    return text

def render(markdown):
    lines=markdown.splitlines();result=[];paragraph=[];listing=False;i=0
    def flush():
        if paragraph:result.append('<p>'+inline(' '.join(paragraph))+'</p>');paragraph.clear()
    while i<len(lines):
        line=lines[i]
        if not line.strip():
            flush()
            if listing:result.append('</ul>');listing=False
            i+=1;continue
        if line.startswith('|'):
            flush();table=[]
            while i<len(lines) and lines[i].startswith('|'):
                cells=[x.strip() for x in lines[i].strip().strip('|').split('|')]
                if not all(re.fullmatch(r':?-+:?',c) for c in cells):table.append(cells)
                i+=1
            result.append('<table>'+''.join('<tr>'+''.join(('<th>' if n==0 else '<td>')+inline(c)+('</th>' if n==0 else '</td>') for c in row)+'</tr>' for n,row in enumerate(table))+'</table>')
            continue
        if line.startswith('#'):
            flush();level=len(line)-len(line.lstrip('#'));result.append(f'<h{level}>'+inline(line[level:].strip())+f'</h{level}>')
        elif line.startswith('- '):
            flush()
            if not listing:result.append('<ul>');listing=True
            result.append('<li>'+inline(line[2:])+'</li>')
        elif line.startswith('!['):flush();result.append(inline(line))
        elif line.startswith('*') and line.endswith('*'):flush();result.append('<p class="caption">'+inline(line.strip('*'))+'</p>')
        else:paragraph.append(line)
        i+=1
    flush()
    if listing:result.append('</ul>')
    return '\n'.join(result)

def main():
    (OUT/'evidence').mkdir(exist_ok=True)
    assets=[]
    for source,dest in [
        ('right_hand_rotation_20261112/lower_grip_actual_review/actual_thumb_push.jpg','images/actual_thumb_push.jpg'),
        ('tip_first_trial_20261112_173936_071900/camera/frame_00148.jpg','images/empty_start_before.jpg'),
        ('tip_first_trial_20261112_173936_071900/camera/frame_00446.jpg','images/empty_start_after.jpg'),
        ('right_hand_rotation_20261112/tip_first_start_actual_review/review.json','evidence/start_review.json'),
        ('right_hand_rotation_20261112/tip_first_empty_initial_scope/trace.json','evidence/empty_path_trace.json'),
        ('github_desktop/DESKTOP_UPLOAD_COMPLETE.json','evidence/upload_receipt.json')]:assets.append(copy(source,dest))
    reference_paths=[
        'leap_cycle_tip_first_transfer.cmd','leap_cycle_tip_first_transfer_20261112.py',
        'requirements-leap.txt','hand_motor_mapping.py',
        'tip_first_trial_plan_20261112.py','tip_first_trial_gate_20261112.py','screen_tip_first_current_20261112.py',
        'right_hand_rotation_20261112/tip_first_trial_v2/READY.json',
        'right_hand_rotation_20261112/tip_first_trial_v2/review.json',
        'right_hand_rotation_20261112/tip_first_trial_v2/operator_guide.md',
        'right_hand_rotation_20261109/policy_audit/STATUS.md',
        'right_hand_rotation_20261109/prehardware_policy_cycle/WORK_STATUS.md',
        'right_hand_rotation_20261109/prehardware_policy_cycle/next_cycle/STATUS.md',
        'team_integration_20261109/middleware_review/REVIEW.md',
        'integration_handoff_20261104.md','professor_progress_20261015.md',
        'lower_grip_trial_20261112_162806_610535/result.json',
        'tip_first_trial_20261112_173345_707995/result.json',
        'tip_first_trial_20261112_173745_907847/result.json',
        'tip_first_trial_20261112_173936_071900/result.json']
    references=[]
    for relative in reference_paths:
        p=ROOT/relative
        if not p.is_file():raise RuntimeError('Missing referenced source: '+relative)
        identical=(CHECKOUT/relative).is_file() and sha(CHECKOUT/relative)==sha(p)
        references.append(dict(path=relative,sha256=sha(p),same_as_published_checkout=identical,github_url=REMOTE+quote(relative,safe='/') if identical else None))
    old=json.loads((ROOT/'right_hand_rotation_20261112/tip_first_trial_v2/review.json').read_bytes())
    unchanged=all(sha(ROOT/p)==old['code_hashes'][p] for p in ['tip_first_trial_plan_20261112.py','tip_first_trial_gate_20261112.py','screen_tip_first_current_20261112.py'])
    assert unchanged,'Reassess implementation status before publishing handoff'
    patched='empty_initial_thumb_index_scope' in old
    inventory=dict(prepared_for_date='2026-11-13',as_of_date='2026-11-13' if patched else '2026-11-12',source_commit=COMMIT,
                   startup_exception_runtime_patch_applied=patched,startup_exception_analysis_completed=True,
                   runtime_sources_still_match_prior_review=True,source_files=references,copied_evidence=assets,
                   new_hardware_access=False,new_github_upload=False)
    (OUT/'evidence/SOURCES.json').write_text(json.dumps(inventory,ensure_ascii=False,indent=2),encoding='utf-8')
    links=['# 바로 여는 원본 자료','',f'고정 커밋: `{COMMIT}`. 아래 링크는 실제 게시된 파일과 로컬 원본 해시가 같은지 확인한 뒤 작성했다.','']
    for item in references:
        if item['github_url']:links.append('- ['+item['path']+']('+item['github_url']+')')
        else:links.append('- `'+item['path']+'` — 원래 작업 폴더에서 확인')
    (OUT/'SOURCE_LINKS.md').write_text('\n'.join(links)+'\n',encoding='utf-8')
    markdown=(OUT/'PROFESSOR_MEETING.md').read_text(encoding='utf-8')
    pages=markdown.split('<!-- PAGEBREAK -->')
    assert len(pages)==2
    style='''
@page{size:A4;margin:15mm 16mm}*{box-sizing:border-box}body{margin:0;background:#e9eef4;color:#172333;font-family:"Malgun Gothic","Noto Sans KR",Arial,sans-serif;font-size:10pt;line-height:1.5}.toolbar{max-width:210mm;margin:16px auto;padding:12px;background:#fff;border-radius:8px}.toolbar button{padding:8px 20px;background:#173b5f;color:#fff;border:0;border-radius:5px;cursor:pointer}.page{width:210mm;min-height:297mm;padding:15mm 16mm;margin:16px auto;background:#fff;box-shadow:0 2px 18px #14253d20}h1{font-size:23pt;line-height:1.25;color:#173b5f;margin:0 0 8mm}h2{font-size:13pt;color:#173b5f;border-bottom:1px solid #ccd9e5;padding-bottom:2mm;margin:5mm 0 2mm}h3{font-size:11pt;margin:5mm 0 2mm}p{margin:2.2mm 0}ul{margin:2mm 0;padding-left:5mm}li{margin:1mm 0}table{border-collapse:collapse;width:100%;font-size:9pt;margin:3mm 0}th,td{border:1px solid #d5dfe9;padding:2.1mm 2.5mm;vertical-align:top}th{background:#edf3f8;text-align:left}th:first-child,td:first-child{width:25mm;white-space:nowrap}code{font-family:Consolas,monospace;font-size:.9em;overflow-wrap:anywhere}figure{margin:3mm 0 1mm;text-align:center}img{display:block;width:104mm;height:58.5mm;object-fit:contain;margin:auto}figcaption{font-size:8pt;color:#526478;margin-top:1mm}.caption{font-size:8pt;color:#526478}a{color:#185a8f}footer{font-size:8pt;color:#65778a;margin-top:4mm;border-top:1px solid #d5dfe9;padding-top:2mm}
@media print{body{background:#fff}.toolbar{display:none}.page{width:auto;min-height:0;margin:0;padding:0;box-shadow:none;break-after:page}.page:last-child{break-after:auto}h2,h3{break-after:avoid}table,figure,p,li{break-inside:avoid}}
'''
    style+='body{font-size:9.4pt;line-height:1.42}h1{font-size:21pt;margin-bottom:5mm}h2{margin:4mm 0 1.5mm}p{margin:1.7mm 0}table{font-size:8.6pt}th,td{padding:1.7mm 2.3mm}img{width:86mm;height:48.4mm}figure{margin:2mm 0 1mm}footer{margin-top:3mm}'
    document='<!doctype html><html lang="ko"><meta charset="utf-8"><title>오른손 큐브 조작 진행 보고 · 2026-11-13</title><style>'+style+'</style><body><div class="toolbar"><button onclick="window.print()">인쇄 / PDF 저장</button>　2026-11-13 교수님 미팅용</div>'
    for i,page in enumerate(pages,1):document+=f'<section class="page" id="page{i}">'+render(page)+f'<footer>오른손 큐브 조작 · 자료 기준 2026-11-12　 |　{i}/2</footer></section>'
    document+='</body></html>'
    (OUT/'PROFESSOR_MEETING.html').write_text(document,encoding='utf-8')
    browsers=[Path(os.environ.get('PROGRAMFILES(X86)','C:/Program Files (x86)'))/'Microsoft/Edge/Application/msedge.exe',Path(os.environ.get('PROGRAMFILES','C:/Program Files'))/'Google/Chrome/Application/chrome.exe']
    browser=next((p for p in browsers if p.is_file()),None)
    pdf=OUT/'PROFESSOR_MEETING.pdf'
    if browser:
        env=os.environ.copy();env.pop('ELECTRON_RUN_AS_NODE',None)
        result=subprocess.run([str(browser),'--headless=new','--disable-gpu','--disable-background-networking','--no-first-run','--no-pdf-header-footer',
            '--user-data-dir='+str(OUT/'.render_profile'),'--print-to-pdf='+str(pdf),(OUT/'PROFESSOR_MEETING.html').as_uri()],capture_output=True,env=env,timeout=90,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        if result.returncode or not pdf.is_file():raise RuntimeError('PDF rendering failed; HTML and Markdown were retained')
        blob=pdf.read_bytes();assert blob.startswith(b'%PDF-')
        page_count=len(re.findall(rb'/Type\s*/Page\b',blob))
        print('PDF_PAGES',page_count,flush=True)
        if page_count!=2:raise RuntimeError('Review print overflow before delivery')
        print('FITZ_AVAILABLE',bool(importlib.util.find_spec('fitz')),flush=True)
        if importlib.util.find_spec('fitz'):
            import fitz
            doc=fitz.open(pdf)
            for i,page in enumerate(doc):page.get_pixmap(matrix=fitz.Matrix(1.1,1.1)).save(str(OUT/f'.preview_page{i+1}.png'))
    print(json.dumps(dict(folder=str(OUT),pdf_created=pdf.exists(),references=len(references),all_reference_files_published=all(r['same_as_published_checkout'] for r in references),startup_patch_applied=patched,hardware_access=False,github_upload=False),ensure_ascii=False),flush=True)

if __name__=='__main__':main()
