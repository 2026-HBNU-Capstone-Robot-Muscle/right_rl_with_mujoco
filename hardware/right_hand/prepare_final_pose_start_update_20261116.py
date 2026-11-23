"""Rebind the new local-only passive start to saved actual read-only values."""
import json
import shutil
from final_grip_pose_plan_20261116 import ROOT,OUT,REQUEST,PROOF,FinalGripPosePlan,sha
from final_pose_start_scope_20261116 import SOURCE

backup=OUT/'before_passive_roots';backup.mkdir(exist_ok=False)
for path in (REQUEST,PROOF,OUT/'runtime_guard_review.json'):
    shutil.copy2(path,backup/path.name)
r=json.loads(SOURCE.read_bytes())
if r['write_events'] or not r['all_torque_off_verified']:
    raise RuntimeError('Passive scope source must be no-write/OFF')
state=r['preflight_state']
request=json.loads(REQUEST.read_bytes());request.update(off_state=state,source=SOURCE.relative_to(ROOT).as_posix(),source_sha256=sha(SOURCE))
REQUEST.write_text(json.dumps(request,indent=2)+'\n',encoding='utf-8')
source=(ROOT/'screen_empty_initial_scope_20261113.py').read_text(encoding='utf-8')
source=source.replace('from loaded_quarter_model_policy_20261025 import correct_model','from final_pose_start_scope_20261116 import correct_model')
needle="if phase in ('initial_hold',ENTRY):\n                        j="
replace="if phase in ('initial_hold',ENTRY):\n                        for physical in (7,11,15):\n                            sim=int(np.flatnonzero(simids==physical)[0])\n                            if raw[physical]<low[physical]:\n                                ex[sim]=max((raw[physical]-4-2048)*TICK-d.qpos[sim]-TICK,d.qpos[sim]-native_high[sim])\n                        j="
if source.count(needle)!=1:raise RuntimeError('Screen layout changed')
source=source.replace(needle,replace)
source=source.replace("if __name__=='__main__':main()",'')
source+='\nfrom final_grip_pose_plan_20261116 import FinalGripPosePlan\nOfflineRecovery=FinalGripPosePlan\nif __name__=="__main__":main()\n'
(ROOT/'screen_final_pose_recovery_20261116.py').write_text(source,encoding='utf-8')
full=source.replace('(ENTRY,RING_CLEAR,RING_MARGIN,ROOTS,OPEN,SETTLE)', '(ENTRY,RING_CLEAR,RING_MARGIN,ROOTS,OPEN,SETTLE,MOVE,HOLD)')
full=full.replace('import FinalGripPosePlan\nOfflineRecovery','import FinalGripPosePlan,MOVE,HOLD\nOfflineRecovery')
(ROOT/'screen_final_grip_pose_full_20261116.py').write_text(full,encoding='utf-8')
p=FinalGripPosePlan(REQUEST);rows=p.plan(p.source_position())
print(json.dumps(dict(start=p.source_position().tolist(),goals=len(rows),seconds=len(rows)*.075,devices_opened=False)))
