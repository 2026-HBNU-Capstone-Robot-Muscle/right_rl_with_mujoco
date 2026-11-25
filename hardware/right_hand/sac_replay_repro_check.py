"""Compare archived teacher replays without running dynamics or opening devices."""
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
a=json.loads((ROOT/'sac_stage0_teacher_v2_result.json').read_text())
b=json.loads((ROOT/'sac_stage0_teacher_v2_result_hold1_offsets0,1.json').read_text())
rows=[]
for rb in b['results']:
    ra=next(x for x in a['results'] if (x['offset_index'],x['seed'])==(rb['offset_index'],rb['seed']))
    differences=[]
    for x,y in zip(ra['trace'],rb['trace']):
        differences.append(dict(step=x['step'],qpos_max_delta=float(np.max(np.abs(np.array(x['qpos'])-y['qpos']))),
                                ctrl_max_delta=float(np.max(np.abs(np.array(x['ctrl'])-y['ctrl'])))))
    rows.append(dict(offset_index=rb['offset_index'],seed=rb['seed'],first_ten=differences[:10],
                     first_qpos_over_1e6=next((x for x in differences if x['qpos_max_delta']>1e-6),None),
                     first_ctrl_over_1e6=next((x for x in differences if x['ctrl_max_delta']>1e-6),None)))
report=dict(status='offline_replay_repeatability_check',rows=rows)
(ROOT/'sac_replay_repro_check_result.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
