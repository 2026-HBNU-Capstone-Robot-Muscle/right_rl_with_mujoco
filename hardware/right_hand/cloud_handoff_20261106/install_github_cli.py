"""Install the official portable GitHub CLI with its published checksum."""
from pathlib import Path
import hashlib
import io
import json
import urllib.request
import zipfile

ROOT=Path(__file__).resolve().parent.parent
DEST=ROOT/'.runtime/github-cli'


def fetch(url):
    request=urllib.request.Request(url,headers={'User-Agent':'xlerobot-local-handoff','Accept':'application/vnd.github+json'})
    with urllib.request.urlopen(request,timeout=90) as response:return response.read()


def main():
    info=json.loads(fetch('https://api.github.com/repos/cli/cli/releases/latest'))
    assets=info['assets']
    binary=next(a for a in assets if a['name'].endswith('_windows_amd64.zip'))
    sums=next(a for a in assets if a['name'].endswith('_checksums.txt'))
    checksums=fetch(sums['browser_download_url']).decode('utf-8')
    expected=next(line.split()[0] for line in checksums.splitlines() if line.split()[-1].lstrip('*')==binary['name'])
    data=fetch(binary['browser_download_url'])
    actual=hashlib.sha256(data).hexdigest()
    if actual!=expected:raise ValueError('Official release checksum mismatch')
    DEST.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(io.BytesIO(data)) as archive:
        entry=next(name for name in archive.namelist() if name.endswith('bin/gh.exe'))
        (DEST/'gh.exe').write_bytes(archive.read(entry))
    (DEST/'release.json').write_text(json.dumps({'tag':info['tag_name'],'asset':binary['name'],'sha256':actual,'source':info['html_url']},indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'version':info['tag_name'],'checksum_verified':True,'installed':str(DEST/'gh.exe')},ensure_ascii=False))


if __name__=='__main__':main()
