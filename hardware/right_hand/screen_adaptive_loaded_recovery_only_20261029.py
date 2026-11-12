"""Independent current EMPTY recovery screen while free-cube cases run."""
import json,sys
from adaptive_yaw_loaded_plan_20261029 import AdaptiveYawLoadedPlan,ROOT,OUT,REQUEST,hashes
import screen_calibrated_empty_margin_20261028 as screen
if __name__=='__main__':
    before=hashes();folder=OUT/'empty_recovery_screen'
    screen.OfflineRecovery=AdaptiveYawLoadedPlan
    sys.argv=['empty','--request',str(REQUEST),'--output-folder',folder.relative_to(ROOT).as_posix()]
    screen.main()
    if hashes()!=before:raise RuntimeError('Control sources changed during EMPTY screen')
    (folder/'control_source_hashes.json').write_text(json.dumps(before,indent=2)+'\n',encoding='utf-8')
