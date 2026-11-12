import json
from pathlib import Path
from quarter_finish_empty_plan_20261025 import motion_goals,OLD_TURN
r=json.loads(Path('loaded_quarter_trial_20261025_171607_019752/result.json').read_bytes());s=[x for x in r['samples'] if x['phase']==OLD_TURN]
print('count',len(s),'keys',s[0].keys(),'first',s[0]['goal'],'last',s[-1]['goal'])
print('first_ref',motion_goals()[0].tolist(),'end_ref',motion_goals()[-1].tolist())
print({k:r.get(k) for k in ('status','loaded_goal_count','last_issued_command','final_command','planned_final_command')})
