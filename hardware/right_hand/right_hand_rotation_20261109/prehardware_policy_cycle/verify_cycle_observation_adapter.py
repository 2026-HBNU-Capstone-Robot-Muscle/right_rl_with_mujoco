"""Check frame composition, units, freshness and existing-health boundaries."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
import json
import hashlib
import numpy as np
from cycle_observation_adapter import CycleObservationAdapter,matrix_quaternion
from shadow_runtime import matrix,TICK

OUT=Path(__file__).resolve().parent


def verify():
    root=OUT/'next_cycle/handoff_refinement_20261110/robust_cycle_preparation_20261111';rng=np.random.default_rng(919);checks=[]
    def R():
        q=rng.normal(size=4);q/=np.linalg.norm(q);return matrix(q)
    for _ in range(128):
        Rwa=R();twa=rng.uniform(-.2,.2,3);Rca=R();tca=rng.uniform(-.2,.2,3);C=rng.uniform(-.1,.1,3);B=R()
        registration=dict(status='synthetic_unverified_fixture',physical_validation_completed=False,R_world_anchor=Rwa.tolist(),t_world_anchor_m=twa.tolist(),policy_palm_position_world_m=[.11,0,.03],cube_side_m=.055,cube_marker_m=.0107,wrist_marker_m=.027)
        adapter=CycleObservationAdapter(registration,simulator_fixture=True);vision=dict(anchor_id=41,cube_pose_frame='cube_center',cube_frame_id=8,anchor_frame_id=8,pose_time=1.,R_camera_anchor=Rca,t_camera_anchor_m=tca,R_camera_cube=Rca@Rwa.T@B,t_camera_cube_m=tca+Rca@Rwa.T@(C-twa))
        command=np.full(16,2048);state=dict(position=command.copy(),goal=command.copy());kwargs=dict(state=state,tracking_rad=np.zeros(16),commanded_ticks=command,checked_by='ForwardHoldBus.checked_sample',read_started_at=1.,read_finished_at=1.01,vision=vision,sequence=5)
        sample=adapter.build(**kwargs);assert np.max(abs(sample['cube_position']-C))<1e-12 and np.max(abs(matrix(sample['cube_quaternion'])-B))<1e-12
        assert not sample['registration']['verified'] and sample['bus_checks_passed']
    checks.append('128 independent camera/anchor/cube frame compositions reproduce world pose and quaternion ordering to1e-12')
    def rejects(call):
        try:call()
        except ValueError:return
        raise AssertionError('Invalid observation was accepted')
    rejects(lambda:CycleObservationAdapter(registration))
    rejects(lambda:CycleObservationAdapter(dict(registration,cube_marker_m=.0106),simulator_fixture=True))
    for replacement in (dict(checked_by='unchecked_raw_snapshot'),dict(read_finished_at=1.21),dict(tracking_rad=np.ones(16)),dict(state=dict(position=command,goal=command+1)),dict(vision=dict(vision,pose_time=.8)),dict(vision=dict(vision,anchor_frame_id=9)),dict(vision=dict(vision,cube_pose_frame='marker_center')),dict(vision=dict(vision,R_camera_anchor=np.zeros((3,3))))):
        rejects(lambda replacement=replacement:adapter.build(**dict(kwargs,**replacement)))
    checks.append('Unverified registration, old marker dimensions, unchecked reads, stale/asynchronous images, invalid rotations, goal mismatch and tracking mismatch are rejected')
    for B in (np.eye(3),np.diag([1,-1,-1]),np.diag([-1,1,-1]),np.diag([-1,-1,1])):assert np.max(abs(matrix(matrix_quaternion(B))-B))<1e-12
    checks.append('Quaternion conversion covers identity and all180-degree coordinate-axis cases')
    report=dict(status='passed',checks=checks,source_sha256=hashlib.sha256((OUT/'cycle_observation_adapter.py').read_bytes()).hexdigest(),tests_are_synthetic_frame_and_contract_tests_not_camera_calibration=True,physical_registration_verified=False,existing_motor_health_checker_reused_not_replaced=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False)
    (root/'observation_adapter_verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8');return report


if __name__=='__main__':print(json.dumps(verify(),ensure_ascii=False),flush=True)
