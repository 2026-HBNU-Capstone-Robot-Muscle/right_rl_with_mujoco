"""Matched-duration controls and OFFLINE regrasp speed hypotheses.

No runtime command/rate/protection setting is edited. Speeds above the existing
two-tick plan are diagnostics only and are never approved for hardware here.
"""
import json
import numpy as np
from original_horizontal_scene_20261025 import Scene,OUT


def main():
    data=json.loads((OUT/'free_cube_regrasp_review_v3.json').read_bytes())
    paths=[r for r in data['paths'] if r['static_check']['clear'] and r['order'] in [
        ['simultaneous'],['ring_together','index','middle','thumb'],
        ['ring_together','thumb','index','middle'],['ring_root_first','thumb','middle','index']]]
    scene=Scene();results=[]
    for r in paths:
        raw=np.array(r['goal_ticks'])
        for factor in (1,2,4,8):
            chosen=np.unique(np.r_[np.arange(0,len(raw),factor),len(raw)-1])
            scaled=raw[chosen]
            before=scene.rollout(np.tile(scene.raw0,(len(scaled),1)),.8,hold_s=3.)
            action=scene.rollout(scaled,.8,hold_s=3.,record=factor==4)
            row=dict(order=r['order'],sample_stride=factor,target_period_s=.05,
                maximum_goal_step_ticks=int(np.max(abs(np.diff(np.vstack([scene.raw0,scaled]),axis=0)))),
                nominal_motion_s=len(scaled)*.05,
                unchanged_hold_control={k:v for k,v in before.items() if k not in ('state','frames')},
                action={k:v for k,v in action.items() if k not in ('state',)},
                goal_ticks=scaled.tolist(),hardware_execution_allowed=False)
            results.append(row)
            print('MATCHED_TIMING',r['order'],'factor',factor,'hold_control_retained',before['hold_retained'],
                'action_retained',action['hold_retained'],'fingers',action['final']['distinct_finger_contact_count'],
                'yaw',round(action['final']['yaw_heading_deg'],2),flush=True)
    good=[r for r in results if r['action']['hold_retained'] and r['action']['final']['retained']
        and r['action']['final']['distinct_finger_contact_count']>=2]
    out=dict(comparisons=results,free_cube_grasp_candidates=good,
        same_initial_state_and_duration_controls=True,current_runtime_goal_step_unchanged=True,
        faster_variants_are_offline_hypotheses_only=True,actual_motor_response_not_identified=True,
        hardware_execution_allowed=False,hardware_commands_sent=0,physical_ninety_degree_rotation_verified=False)
    (OUT/'regrasp_timing_control_review.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
    print('SUMMARY','free_cube_grasp_candidates',len(good),flush=True)


if __name__=='__main__':main()
