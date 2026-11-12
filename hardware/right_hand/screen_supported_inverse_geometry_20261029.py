"""Bind the whole201-condition static review to the exact new path bytes."""
import hashlib,json,sys
import screen_loaded_yaw_static_20261027 as screen
from supported_inverse_empty_plan_20261029 import ROOT,OUT,HAND_PATH
def main():
    before=hashlib.sha256(HAND_PATH.read_bytes()).hexdigest();saved=sys.argv
    try:
        sys.argv=['static','--path',HAND_PATH.relative_to(ROOT).as_posix(),'--output-folder',(OUT/'whole_geometry').relative_to(ROOT).as_posix(),'--all-offsets']
        screen.main()
    finally:sys.argv=saved
    if before!=hashlib.sha256(HAND_PATH.read_bytes()).hexdigest():raise RuntimeError('Hand path changed during static review')
    path=OUT/'whole_geometry/review.json';r=json.loads(path.read_bytes());r['source_sha256']=before
    path.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
