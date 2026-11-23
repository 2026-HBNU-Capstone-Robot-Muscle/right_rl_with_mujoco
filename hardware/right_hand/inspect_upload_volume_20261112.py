"""Read package metadata only; never stage, commit or push."""
import json
import zipfile
from pathlib import Path
root=Path(__file__).resolve().parent
archive=root/'full_project_delivery_20261112/RIGHT_HAND_FULL_GITHUB_UPLOAD_20261112.zip'
with zipfile.ZipFile(archive) as z:
    manifest=json.loads(z.read('RIGHT_HAND_FULL_MANIFEST.json'))
    seen=set();compressed=0;raw=0;groups={}
    for item in manifest['files']:
        digest=item['sha256']
        if digest in seen:continue
        seen.add(digest)
        entry=z.getinfo('hardware/right_hand/'+item['path'])
        compressed+=entry.compress_size;raw+=entry.file_size
        group=item['path'].split('/')[0]
        groups[group]=groups.get(group,0)+entry.compress_size
    print(json.dumps(dict(unique_source_blobs=len(seen),unique_compressed_zip_bytes=compressed,unique_raw_bytes=raw,estimate_only=True),ensure_ascii=False))
    print('largest_folders',json.dumps(sorted(groups.items(),key=lambda x:x[1],reverse=True)[:12]))
