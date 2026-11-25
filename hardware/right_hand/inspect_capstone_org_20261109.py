"""Read the user-linked organization's accessible repositories, no mutations."""
from pathlib import Path
import json,subprocess

ROOT=Path(__file__).resolve().parent
GH=ROOT/'.runtime/github-cli/gh.exe'
ORG='2026-HBNU-Capstone-Robot-Muscle'


def api(endpoint):
    result=subprocess.run([str(GH),'api',endpoint],text=True,encoding='utf-8',stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    if result.returncode:return None,result.stderr.strip()
    return json.loads(result.stdout),None


def main():
    details,error=api('orgs/'+ORG)
    if error:
        print(json.dumps({'organization':ORG,'readable':False,'error':error},ensure_ascii=False));return
    repositories=[];page=1
    while True:
        rows,error=api(f'orgs/{ORG}/repos?per_page=100&page={page}&type=all')
        if error:break
        repositories.extend({'full_name':row['full_name'],'private':row['private'],'html_url':row['html_url'],
                             'default_branch':row['default_branch'],'permissions':row.get('permissions',{}),'description':row.get('description')}
                            for row in rows)
        if len(rows)<100:break
        page+=1
    result={'organization':ORG,'page_readable':True,'repositories_visible_to_current_login':repositories,'list_error':error}
    for repo in repositories[:3]:
        content,read_error=api('repos/'+repo['full_name']+'/contents')
        repo['root_read_error']=read_error
        if not read_error:repo['root_files']=[entry['name'] for entry in content] if isinstance(content,list) else []
    folder=ROOT/'right_hand_rotation_20261109';folder.mkdir(exist_ok=True)
    (folder/'organization_access_check.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False))


if __name__=='__main__':main()
