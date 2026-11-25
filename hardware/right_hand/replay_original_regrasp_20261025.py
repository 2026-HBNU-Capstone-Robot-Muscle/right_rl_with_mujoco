"""Test IK regrasp candidates against free-cube dynamics, no devices."""
import itertools
import argparse
import json
import numpy as np
from original_horizontal_scene_20261025 import Scene,interpolate,OUT


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--ring-clear',action='store_true');args=parser.parse_args()
    input_name='regrasp_configuration_search_v3.json' if args.ring_clear else 'regrasp_configuration_search_v2.json'
    inputs=json.loads((OUT/input_name).read_bytes())
    scene=Scene();scene.reset_baseline(.8)
    limits=json.loads((OUT.parent.parent/'right_hand_rotation_20261023/native_motor_limits.json').read_bytes())
    lower=np.ceil(limits['lower_ticks']).astype(int);upper=np.floor(limits['upper_ticks']).astype(int)
    groups={'index':[4,5,6,7],'middle':[8,9,10,11],'thumb':[0,1,2,3]}
    candidates=[r for r in inputs['candidates'] if r['geometric_grasp_candidate']]
    results=[]
    for index,candidate in enumerate(candidates):
        target=np.minimum(np.maximum(np.array(candidate['goal_ticks']),lower),upper)
        orders=[('simultaneous',)]+list(itertools.permutations(groups))
        if args.ring_clear:
            orders=[('simultaneous',)]+[('ring_'+style,)+p for style in ('together','root_first') for p in itertools.permutations(groups)]
        for order in orders:
            if order==('simultaneous',):raw=interpolate(scene.raw0,target)
            else:
                pieces=[];previous=scene.raw0
                for group in order:
                    if group=='ring_root_first':
                        for joint in (15,13,14,12):
                            piece=interpolate(previous,target,[joint]);pieces.append(piece);previous=piece[-1]
                    else:
                        indices=[12,13,14,15] if group=='ring_together' else groups[group]
                        piece=interpolate(previous,target,indices);pieces.append(piece);previous=piece[-1]
                raw=np.vstack(pieces)
            assert np.array_equal(raw[-1],target)
            static=scene.static_check(raw)
            row=dict(candidate=index,order=order,samples=len(raw),nominal_motion_s=len(raw)*.05,
                static_check=static,goal_ticks=raw.tolist())
            if static['clear']:
                run=scene.rollout(raw,.8,hold_s=3.)
                row['nominal']={k:v for k,v in run.items() if k not in ('state','frames')}
                print('FREE_REGRASP',order,'contacts',run['final']['hand_contacts'],
                    'retained',run['hold_retained'],'yaw',round(run['final']['yaw_heading_deg'],2),flush=True)
            else:print('REGRASP_PATH_REJECTED',order,'overlap_mm',round(static['maximum_model_overlap_mm'],3),flush=True)
            results.append(row)
    accepted=[r for r in results if r['static_check']['clear'] and r['nominal']['hold_retained']
        and r['nominal']['final']['retained'] and r['nominal']['final']['distinct_finger_contact_count']>=2]
    accepted.sort(key=lambda r:r['nominal']['final']['ninety_target_error_deg'])
    sensitivity=[]
    if accepted:
        for f in (.4,.8,1.2):
            run=scene.rollout(np.array(accepted[0]['goal_ticks']),f,record=f==.8)
            sensitivity.append(dict(friction=f,**{k:v for k,v in run.items() if k!='state'}))
    report=dict(source=input_name,paths=results,
        simulated_regrasp_candidates=accepted,sensitivity=sensitivity,
        cube_never_attached_during_dynamic_rollout=True,hand_trajectory_prescribed=True,
        actual_motor_response_identified=False,hardware_execution_allowed=False,
        hardware_commands_sent=0,physical_grasp_or_ninety_rotation_verified=False)
    output_name='free_cube_regrasp_review_v3.json' if args.ring_clear else 'free_cube_regrasp_review.json'
    (OUT/output_name).write_text(json.dumps(report,indent=2),encoding='utf-8')
    print('SUMMARY','free_cube_regrasp_candidates',len(accepted),flush=True)


if __name__=='__main__':main()
