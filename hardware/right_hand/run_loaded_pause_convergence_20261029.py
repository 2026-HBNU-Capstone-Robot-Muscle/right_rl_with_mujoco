"""Compare the two failing parameter pairs at1ms and0.5ms."""
import json,sys
from adaptive_yaw_loaded_plan_20261029 import ROOT,OUT,STRESS_PATH
import screen_loaded_yaw_convergence_20261029 as screen
if __name__=='__main__':
    results=[]
    for dt in (.001,.0005):
        for gain,friction in ((1.5,.2),(1.,.3)):
            folder=OUT/f'convergence_dt{dt:g}_gain{gain:g}_mu{friction:g}'
            sys.argv=['convergence','--candidate',STRESS_PATH.relative_to(ROOT).as_posix(),'--output-folder',folder.relative_to(ROOT).as_posix(),
                '--front','-10','--dt',str(dt),'--gain',str(gain),'--friction',str(friction)]
            screen.main()
            results.append(json.loads((folder/'review.json').read_bytes()))
    (OUT/'numerical_convergence.json').write_text(json.dumps(dict(cases=results,all_passed=all(r['all_passed'] for r in results),devices_opened=False),indent=2)+'\n',encoding='utf-8')
