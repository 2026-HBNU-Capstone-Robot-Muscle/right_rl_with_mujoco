"""Check operator links, frozen sources, unchanged limits and rendered media."""
import hashlib,json,re,subprocess
from pathlib import Path
from middle_probe_plan_20261106 import R,O,CANDIDATE,PROOF,hashes,sha
from validate_taught_grasp_artifacts_20261106 import PROTECTED
def main():
    h=json.loads((O/'handoff.json').read_bytes());ready=json.loads((O/'ready.json').read_bytes());manifest=json.loads((O/'evidence/manifest.json').read_bytes());assert h['proof_sha256']==sha(PROOF) and ready['proof_sha256']==sha(PROOF) and h['candidate_sha256']==manifest['candidate_sha256']==sha(CANDIDATE) and ready['code_hashes']==hashes()
    for name,digest in PROTECTED.items():assert sha(R/name)==digest,name
    for name,digest in manifest['files'].items():assert sha(O/'evidence'/name)==digest,name
    links=[]
    for page in (O/'operator_guide.md',O/'evidence/index.html'):
        text=page.read_text(encoding='utf-8');targets=re.findall(r'\]\(([^)]+)\)',text) if page.suffix=='.md' else re.findall(r'(?:src|href)="([^"]+)"',text)
        for target in targets:
            if target.startswith(('http:','https:','#')):continue
            path=(page.parent/target).resolve();path.relative_to(R)
            if not path.is_file():raise RuntimeError('Broken operator link '+target)
            links.append(path.relative_to(R).as_posix())
    data=json.loads(subprocess.check_output(['ffprobe','-v','error','-select_streams','v:0','-show_entries','stream=codec_name,width,height,nb_frames,r_frame_rate,duration','-of','json',str(O/'evidence/middle_probe_1x.mp4')],text=True));stream=data['streams'][0];assert stream['codec_name']=='h264' and (stream['width'],stream['height'])==(960,720) and int(stream['nb_frames'])==150 and stream['r_frame_rate']=='8/1' and abs(float(stream['duration'])-18.75)<.001
    report=dict(status='middle_probe_operator_handoff_verified',frozen_source_bindings=len(ready['code_hashes']),protected_sources_unchanged=len(PROTECTED),operator_links_checked=len(links),render_files_checked=len(manifest['files']),video=stream,proof_sha256=sha(PROOF),candidate_sha256=sha(CANDIDATE),diagnostic_only=True,physical_trial_executed=False,devices_opened=False)
    (O/'handoff_verification.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report),flush=True)
if __name__=='__main__':main()
