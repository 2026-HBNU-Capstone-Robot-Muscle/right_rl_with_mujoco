"""Whole-cube forward quarter-turn candidate, free cube, offline only."""
import json
import hashlib
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from original_horizontal_scene_20261025 import Scene,OUT


def main():
    p=OUT/'CAD_adapted_forward_reference_path_v2.json';blob=p.read_bytes();plan=json.loads(blob)
    assert plan['full_static_path_clearance_satisfied'] and plan['last_target_reached']
    raw=np.array(plan['dense_goal_ticks']);assert np.max(abs(np.diff(raw,axis=0)))<=2
    previous=json.loads((OUT.parent/'original_sequence_review/visualized_replay.json').read_bytes())
    placement=previous['placement'];scene=Scene(model_kind='CAD_tip',initial_raw=raw[:1],placement=placement)
    static=scene.static_check(raw);assert static['clear']
    results=[]
    for f in (.4,.8,1.2):
        scene.reset_baseline(f)
        R0=scene.caches[f]['R0'];orange=R0@np.array([-1.,0,0])
        front=orange-scene.up*(scene.up@orange);front/=np.linalg.norm(front)
        goal=np.column_stack([scene.up,-front,np.cross(scene.up,-front)])
        scene.caches[f]['target']=goal
        for mode,scale in (('prescribed',1.),('native_motor',.2),('native_motor',.5),('native_motor',1.)):
            control=scene.rollout(np.tile(raw[0],(len(raw)-1,1)),f,hold_s=4.,
                prescribed_hand=mode=='prescribed',gain_scale=scale)
            result=scene.rollout(raw[1:],f,hold_s=4.,record=f==.8 and (mode=='prescribed' or scale==.2),
                prescribed_hand=mode=='prescribed',gain_scale=scale)
            R=Rotation.from_quat(np.array(result['final']['cube_quat_wxyz'])[[1,2,3,0]]).as_matrix()
            actual_rotation=Rotation.from_matrix(R@R0.T).magnitude()
            contacts=[]
            for c in scene.d.contact:
                a,b=int(c.geom1),int(c.geom2)
                if scene.cube in (a,b) and c.dist<.0003:
                    other=b if a==scene.cube else a;contacts.append(scene.m.geom(other).name)
            external=any(n=='floor' or n.startswith('support_') for n in contacts)
            red_error=float(np.degrees(np.arccos(np.clip((R@np.array([1.,0,0]))@scene.up,-1,1))))
            green_front_error=float(np.degrees(np.arccos(np.clip((R@np.array([0.,-1.,0]))@front,-1,1))))
            row=dict(friction=f,mode=mode,native_gain_scale_hypothesis=scale,
                actual_motor_parameters_identified=False,
                red_up_error_deg=red_error,green_front_error_deg=green_front_error,
                whole_rotation_angle_deg=float(np.degrees(actual_rotation)),external_cube_contact=external,
                final= result['final'],hold_retained=result['hold_retained'],
                maximum_joint_tracking_rad=result['maximum_joint_tracking_rad'],
                maximum_hand_self_model_overlap_mm=result['maximum_hand_self_model_overlap_mm'],
                maximum_hand_support_model_overlap_mm=result['maximum_hand_support_model_overlap_mm'],
                native_actuator_force_limits_enabled=result['native_actuator_force_limits_enabled'],
                matched_duration_unchanged_goal_control={k:v for k,v in control.items() if k not in ('state','frames')},
                simulated_target_reached=bool(result['hold_retained'] and result['final']['retained']
                    and result['final']['ninety_target_error_deg']<15 and not external),
                nominal_seconds=(len(raw)-1)*.05,frames=result['frames'])
            results.append(row)
            print('FORWARD_90',f,mode,scale,'target_error',round(result['final']['ninety_target_error_deg'],2),
                'angle',round(row['whole_rotation_angle_deg'],2),'held',row['hold_retained'],
                'target_reached',row['simulated_target_reached'],flush=True)
    report=dict(source_plan=p.name,source_sha256=hashlib.sha256(blob).hexdigest(),
        model=scene.manifest,static_check=static,runs=results,
        goal='whole cube forward quarter turn: red up, green toward initial orange front',
        cube_goal_never_written_into_simulated_physics=True,hardware_execution_allowed=False,
        hardware_commands_sent=0,physical_ninety_degree_rotation_verified=False)
    (OUT/'CAD_forward_quarter_turn_evaluation.json').write_text(json.dumps(report,indent=2),encoding='utf-8')


if __name__=='__main__':main()
