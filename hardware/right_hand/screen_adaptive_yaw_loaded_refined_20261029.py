"""One support-front block of the complete1ms pause-stress model grid."""
import argparse,json,sys
from adaptive_yaw_loaded_plan_20261029 import ROOT,OUT,STRESS_PATH,hashes
import screen_loaded_yaw_convergence_20261029 as screen
if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--front',type=int,choices=(-15,-10,-5),required=True);a=parser.parse_args()
    before=hashes();folder=OUT/f'loaded_pause_1ms_front{a.front}'
    sys.argv=['refined','--candidate',STRESS_PATH.relative_to(ROOT).as_posix(),'--output-folder',folder.relative_to(ROOT).as_posix(),'--front',str(a.front),'--dt','.001']
    screen.main()
    if before!=hashes():raise RuntimeError('Sources changed during1ms grid')
    (folder/'control_source_hashes.json').write_text(json.dumps(before,indent=2)+'\n',encoding='utf-8')
