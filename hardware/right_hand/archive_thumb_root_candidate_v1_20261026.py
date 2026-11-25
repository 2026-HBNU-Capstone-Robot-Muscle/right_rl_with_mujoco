"""Preserve the first candidate and its failed weak-response evidence."""
import hashlib,json,shutil
from thumb_clear_root_empty_plan_20261026 import ROOT,FIXTURE,FILES
def main():
    dest=FIXTURE/'code_snapshot';dest.mkdir(exist_ok=True);hashes={}
    for n in (*FILES,'build_thumb_clear_root_empty_mode_20261026.py','test_thumb_clear_root_empty_20261026.py'):
        p=ROOT/n;shutil.copyfile(p,dest/n);hashes[n]=hashlib.sha256(p.read_bytes()).hexdigest()
    (dest/'hashes.json').write_text(json.dumps(hashes,indent=2),encoding='utf-8')
    print('Archived first candidate; native weak response rejected; no hardware executed.')
if __name__=='__main__':main()
