import json
from pathlib import Path
import numpy as np
from hand_motor_mapping import ticks_to_sim, SIM_TO_MOTOR_IDS

root=Path(__file__).resolve().parent
data=json.loads((root/'right_hand_rotation_20261024/original_sequence_clearance/adapted_reference_path.json').read_bytes())
for row in data['endpoint_reviews']:
    print('POSE',row['stage'],'delta_deg',row['review']['maximum_teacher_deviation_deg'])
for row in data['sparse_failures']:
    print('FAILURE',json.dumps(row))
bad=data['dense_path_overlap_samples']
print('DENSE_BAD',len(bad),'WORST',sorted(bad,key=lambda r:r['model_overlap_mm'],reverse=True)[:8])
v2path=root/'right_hand_rotation_20261024/original_sequence_clearance/adapted_reference_path_v2.json'
if v2path.exists():
    v2=json.loads(v2path.read_bytes());goals=np.array(v2['sparse_path_goal_ticks']);bounds=[0]
    for a,b in zip(goals[:-1],goals[1:]):bounds.append(bounds[-1]+max(1,int(np.ceil(np.max(abs(b-a))/2))))
    for stage in range(1,7):
        lo=bounds[(stage-1)*32];hi=bounds[stage*32]
        bad=[v for v in v2['dense_path_overlap_samples'] if lo<=v['index']<=hi]
        print('V2_STAGE',stage,'range',lo,hi,'bad',len(bad),'max_model_overlap_mm',max([b['model_overlap_mm'] for b in bad] or [0]))
    selected=np.array(v2['dense_goal_ticks'])[bounds[64]:bounds[128]+1]
    bad=[v for v in v2['dense_path_overlap_samples'] if bounds[64]<=v['index']<=bounds[128]]
    out=dict(source='adapted_reference_path_v2.json',teacher_pose_start=2,teacher_pose_end=4,
        original_primitive='thumb push and thumb transfer; historical horizontal motion reference',
        goal_ticks=selected.tolist(),control_dt_s=.05,nominal_seconds=(len(selected)-1)*.05,
        middle_ring_hulls_included=True,static_path_clearance_satisfied=not bad,
        remaining_overlap_samples=bad,hardware_execution_allowed=False,
        cube_dynamics_tested=False,physical_ninety_degree_rotation_verified=False)
    (v2path.parent/'horizontal_reference_primitive.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
    print('HORIZONTAL',len(selected),'duration',out['nominal_seconds'],'clear',not bad)
    extended=np.array(v2['dense_goal_ticks'])[bounds[64]:bounds[192]+1]
    extended_bad=[v for v in v2['dense_path_overlap_samples'] if bounds[64]<=v['index']<=bounds[192]]
    extended_out=dict(out,teacher_pose_end=6,goal_ticks=extended.tolist(),
        nominal_seconds=(len(extended)-1)*.05,static_path_clearance_satisfied=not extended_bad,
        remaining_overlap_samples=extended_bad)
    (v2path.parent/'horizontal_reference_primitive_to6.json').write_text(json.dumps(extended_out,indent=2),encoding='utf-8')
    print('EXTENDED_HORIZONTAL',len(extended),'duration',extended_out['nominal_seconds'],'clear',not extended_bad)
