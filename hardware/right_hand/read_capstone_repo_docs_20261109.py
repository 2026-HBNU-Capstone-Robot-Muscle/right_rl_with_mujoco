"""Cache selected public project READMEs for read-only design review."""
from pathlib import Path
import json,subprocess,base64,hashlib

ROOT=Path(__file__).resolve().parent
GH=ROOT/'.runtime/github-cli/gh.exe'
ORG='2026-HBNU-Capstone-Robot-Muscle'
OUT=ROOT/'right_hand_rotation_20261109/team_sources'


def api(endpoint):
    result=subprocess.run([str(GH),'api',endpoint],text=True,encoding='utf-8',stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    if result.returncode:return None,result.stderr.strip()
    return json.loads(result.stdout),None


def main():
    OUT.mkdir(parents=True,exist_ok=True);reports=[]
    for name in ('right_rl_with_mujoco','ruka-model','left-hand-model','left-rl-with-mujoco','middleware-bridge'):
        (OUT/name).mkdir(exist_ok=True)
        repo=ORG+'/'+name;details,error=api('repos/'+repo+'/readme');row={'repo':repo,'read_error':error}
        if not error:
            content=base64.b64decode(details['content']);folder=OUT/name;folder.mkdir(exist_ok=True)
            (folder/'README.md').write_bytes(content);row.update(path=details['path'],sha256=hashlib.sha256(content).hexdigest(),bytes=len(content))
            print(json.dumps({'repo':repo,'readme':content.decode('utf-8',errors='replace')[:12000]},ensure_ascii=False))
        tree,tree_error=api('repos/'+repo+'/git/trees/main?recursive=1');row['tree_error']=tree_error
        if not tree_error:
            row['files']=[{'path':item['path'],'bytes':item.get('size')} for item in tree['tree'] if item['type']=='blob']
            (OUT/name/'tree.json').write_text(json.dumps(row['files'],indent=2)+'\n',encoding='utf-8')
        reports.append(row)
        print(json.dumps({'repo':repo,'read_error':row['read_error'],'tree_error':row['tree_error'],'file_count':len(row.get('files',[]))},ensure_ascii=False))
    (OUT/'source_index.json').write_text(json.dumps(reports,indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':main()
