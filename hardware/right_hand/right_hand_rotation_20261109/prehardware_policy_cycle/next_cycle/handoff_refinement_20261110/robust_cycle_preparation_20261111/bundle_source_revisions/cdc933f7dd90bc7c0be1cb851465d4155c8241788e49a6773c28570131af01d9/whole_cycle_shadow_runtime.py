"""Transactional whole-cycle proposals from encoders and timestamped object pose.

This module has no motor/serial/camera interface. The first1700 decisions reuse
the existing SAC. Later commands follow the validated teacher reference with
encoder-based MF clearance proposals and pose-based phase-speed selection.
Neither the teacher suffix nor the kinematic shield is labeled SAC learning.
"""
from pathlib import Path
import hashlib
import json
import numpy as np
from shadow_runtime import vector,quaternion,matrix,SIM_IDS,TICK
from staged_shadow_runtime import AdmissibleShadowController
from cycle_joint_clearance import FingerClearance


class WholeCycleShadowController:
    def __init__(self,bundle,observation_mode='registered_observer'):
        self.folder=Path(bundle);self.meta=json.loads((self.folder/'bundle.json').read_text(encoding='utf-8'))
        if self.meta.get('hardware_execution_allowed') is not False:raise ValueError('Only no-output research bundles are supported')
        if observation_mode not in ('registered_observer','simulator_fixture'):raise ValueError('Explicit observation mode required')
        self.observation_mode=observation_mode
        for name,digest in self.meta['files_sha256'].items():
            if hashlib.sha256((self.folder/name).read_bytes()).hexdigest()!=digest:raise ValueError('Bundle input changed: '+name)
        z=np.load(self.folder/'reference.npz',allow_pickle=False);self.goals=z['goals'];self.start=z['start'];self.ref_qpos=z['qpos'];self.low=z['lower'];self.high=z['upper']
        if self.goals.shape!=(5089,16) or self.ref_qpos.shape!=(5090,23):raise ValueError('Unexpected validated reference shape')
        self.prefix=AdmissibleShadowController(self.folder/'sac_prefix');self.clearance=FingerClearance(self.folder/'kinematics.npz')
        self.phase=0;self.sequence=-1;self.last_time=None;self.last_goal=None;self.pending=None;self.baseline=self.start.copy();self.hold_entry=None

    def _validate(self,sample):
        seq=sample.get('sequence');now=float(sample.get('sample_time',float('nan')));pose_time=float(sample.get('pose_time',float('nan')))
        if not isinstance(seq,(int,np.integer)) or seq<=self.sequence:raise ValueError('Fresh sample sequence required')
        if not np.isfinite([now,pose_time]).all() or pose_time>now or now-pose_time>.15:raise ValueError('Fresh timestamped cube pose required')
        if self.last_time is not None and now-self.last_time<.075-1e-8:raise ValueError('Control observations must be at least75ms apart')
        if sample.get('frame_id')!='leap_sim_world' or sample.get('palm_point')!='policy_palm_position':raise ValueError('Declared training sensor coordinates required')
        if self.observation_mode=='registered_observer':
            proof=sample.get('registration',{})
            if proof.get('verified') is not True or proof.get('registration_id')!=self.meta.get('accepted_registration_id') or self.meta.get('accepted_registration_id') is None:
                raise ValueError('A verified current camera-to-policy registration is required')
            if sample.get('bus_checks_passed') is not True:raise ValueError('Existing bus current/voltage/tracking checks must pass')
        elif sample.get('observation_origin')!='simulator_fixture':raise ValueError('Simulator fixture data must be explicitly labeled')
        measured=vector(sample['measured_ticks'],16,'measured ticks');command=vector(sample['commanded_ticks'],16,'commanded ticks');cube=vector(sample['cube_position'],3,'cube position');q=quaternion(sample['cube_quaternion']);vector(sample['palm_position'],3,'palm position')
        if not np.array_equal(command,np.rint(command)):raise ValueError('Integer acknowledged commands required')
        if np.any(command<self.low) or np.any(command>self.high):raise ValueError('Acknowledged command outside native limits')
        if self.last_goal is not None and not np.array_equal(command,self.last_goal):raise ValueError('Acknowledgement differs from committed command')
        if np.max(abs(measured-command))*TICK>.18:raise ValueError('Measured tracking error exceeds existing0.18rad gate')
        return seq,now,measured,command,cube,q

    def propose(self,sample):
        if self.pending is not None:raise ValueError('Commit the pending no-output proposal first')
        seq,now,measured,command,cube,q=self._validate(sample)
        if self.phase<1700:
            result=self.prefix.propose(sample)
            if result['state']=='warming':self.sequence=seq;self.last_time=now;return dict(result,stage='SAC_prefix',hardware_execution_allowed=False)
            goal=np.asarray(result['goal_ticks'],int);next_phase=self.phase+1;baseline=self.goals[self.phase].copy();details=dict(source='existing_SAC_reference_residual',clearance_proposal_applied=False)
        else:
            index=min(self.phase,len(self.goals)-1);reference=self.ref_qpos[index];position_error=float(np.linalg.norm(cube-reference[-7:-4])*1000);R=matrix(q);targetR=matrix(quaternion(reference[-4:]));orientation_error=float(np.degrees(np.arccos(np.clip((np.trace(targetR@R.T)-1)/2,-1,1))))
            limit=1 if self.meta['phase_speed_mode']=='fixed1' or self.meta['phase_speed_mode']=='pose_adaptive' and (position_error>2. or orientation_error>3.) else 2
            desired=self.goals[index];baseline=self.baseline+np.clip(desired-self.baseline,-limit,limit)
            next_phase=self.phase+1 if np.array_equal(baseline,desired) and self.phase<len(self.goals) else self.phase
            goal,shield=self.clearance.select(measured,command,baseline,self.low,self.high,self.meta['MF_static_margin_mm'])
            details=dict(source='saved_teacher_with_encoder_clearance_and_pose_phase_speed',pose_position_error_mm=position_error,pose_orientation_error_deg=orientation_error,phase_rate_ticks=limit,clearance=shield)
        if np.max(abs(goal-command))>2 or np.any(goal<self.low) or np.any(goal>self.high):raise ValueError('New cycle goal violates unchanged command limits')
        stage='first_hold' if 3351<=self.phase<3405 else ('final_hold' if self.phase>=5035 else ('first_rotation' if self.phase<3351 else 'second_rotation'))
        self.pending=dict(sequence=seq,time=now,goal=goal.copy(),next_phase=next_phase,baseline=baseline.copy(),prefix=self.phase<1700)
        self.sequence=seq;self.last_time=now
        return dict(state='shadow_'+stage,stage=stage,reference_index=self.phase,sequence=seq,goal_ticks=goal.tolist(),details=details,motor_commands_sent=0,hardware_execution_allowed=False)

    def commit_shadow(self,sequence,goal_ticks):
        if self.pending is None or sequence!=self.pending['sequence']:raise ValueError('Pending sequence mismatch')
        goal=vector(goal_ticks,16,'acknowledged goal')
        if not np.array_equal(goal,self.pending['goal']):raise ValueError('Acknowledgement changed the proposed goal')
        if self.pending['prefix']:self.prefix.commit_shadow(sequence,goal)
        self.phase=self.pending['next_phase'];self.baseline=self.pending['baseline'];self.last_goal=goal.copy();self.pending=None

    def completion(self):
        return dict(reference_finished=self.phase>=len(self.goals),next_rotation_or_left_hand_authorized=False,final_goal_is_kept=True,automatic_open=False,automatic_torque_off=False,motor_commands_sent=0,hardware_execution_allowed=False)
