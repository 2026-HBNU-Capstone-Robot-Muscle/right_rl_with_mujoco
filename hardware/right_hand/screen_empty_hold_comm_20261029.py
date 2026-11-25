"""Same recovery screen, with its final fixed-goal dynamics extended to45s."""
import argparse,hashlib,json,sys
from pathlib import Path
from empty_hold_comm_plan_20261029 import ROOT,HOLD_SECONDS,EmptyHoldCommPlan
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--request',required=True);parser.add_argument('--output-folder',required=True);a=parser.parse_args()
    source=ROOT/'screen_calibrated_empty_margin_20261028.py';text=source.read_text(encoding='utf-8')
    old="('final_hold',rows[-1][1],1.5)";new=f"('final_hold',rows[-1][1],{HOLD_SECONDS!r})"
    if text.count(old)!=1:raise RuntimeError('Original final-hold screen changed')
    namespace={'__file__':str(source),'__name__':'derived_empty_hold_screen'}
    exec(compile(text.replace(old,new),str(source),'exec'),namespace)
    namespace['OfflineRecovery']=EmptyHoldCommPlan
    saved=sys.argv
    try:sys.argv=['empty','--request',a.request,'--output-folder',a.output_folder];namespace['main']()
    finally:sys.argv=saved
    output=(ROOT/a.output_folder).resolve()/'review.json';r=json.loads(output.read_bytes())
    r.update(fixed_final_hold_seconds=HOLD_SECONDS,original_screen_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        only_screen_change='final finite-dynamics hold1.5s extended to45s; all thresholds unchanged')
    output.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
