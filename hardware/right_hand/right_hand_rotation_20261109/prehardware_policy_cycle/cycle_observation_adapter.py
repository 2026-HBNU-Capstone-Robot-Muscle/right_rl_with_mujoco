"""Read-only bridge from checked motor state and marker poses to policy inputs.

No serial/camera API is imported. The caller must use the existing bus health
checker; this adapter neither replaces its limits nor changes motor settings.
Existing approximate registration assets remain unverified.
"""
import hashlib
import json
import numpy as np
from shadow_runtime import TICK,vector,quaternion


def proper_rotation(value,name):
    R=np.asarray(value,float)
    if R.shape!=(3,3) or not np.isfinite(R).all() or np.max(abs(R.T@R-np.eye(3)))>1e-6 or abs(np.linalg.det(R)-1)>1e-6:raise ValueError(name+' must be a proper finite rotation')
    return R


def matrix_quaternion(R):
    R=proper_rotation(R,'cube orientation');q=np.empty(4);trace=np.trace(R)
    if trace>0:
        s=2*np.sqrt(trace+1);q[:]=[s/4,(R[2,1]-R[1,2])/s,(R[0,2]-R[2,0])/s,(R[1,0]-R[0,1])/s]
    else:
        i=int(np.argmax(np.diag(R)));j=(i+1)%3;k=(i+2)%3;s=2*np.sqrt(1+R[i,i]-R[j,j]-R[k,k]);q[0]=(R[k,j]-R[j,k])/s;q[i+1]=s/4;q[j+1]=(R[j,i]+R[i,j])/s;q[k+1]=(R[k,i]+R[i,k])/s
    q=quaternion(q)
    return q if q[0]>=0 else -q


class CycleObservationAdapter:
    def __init__(self,registration,*,simulator_fixture=False):
        self.registration=dict(registration);self.fixture=bool(simulator_fixture)
        self.Rwa=proper_rotation(registration['R_world_anchor'],'anchor registration');self.twa=vector(registration['t_world_anchor_m'],3,'anchor registration position')
        self.palm=vector(registration['policy_palm_position_world_m'],3,'policy palm sensor position')
        self.registration_id=hashlib.sha256(json.dumps(registration,sort_keys=True,separators=(',',':')).encode()).hexdigest()
        if registration.get('cube_side_m')!=.055 or registration.get('cube_marker_m')!=.0107 or registration.get('wrist_marker_m')!=.027:raise ValueError('Use explicitly reviewed current marker and cube dimensions')
        self.verified=registration.get('status')=='verified_camera_policy_registration' and registration.get('physical_validation_completed') is True and bool(registration.get('evidence_sha256'))
        if not self.fixture and not self.verified:raise ValueError('Current physical camera-to-policy registration is unverified')

    def build(self,*,state,tracking_rad,commanded_ticks,checked_by,read_started_at,read_finished_at,vision,sequence):
        if checked_by!='ForwardHoldBus.checked_sample':raise ValueError('Existing full bus health checker must precede this adapter')
        start=float(read_started_at);now=float(read_finished_at)
        if not np.isfinite([start,now]).all() or not 0<=now-start<=.2:raise ValueError('Original200ms motor-read budget exceeded')
        measured=vector(state['position'],16,'motor positions');goal=vector(state['goal'],16,'goal readback');command=vector(commanded_ticks,16,'acknowledged target');tracking=vector(tracking_rad,16,'checked tracking error')
        if not np.array_equal(goal,command):raise ValueError('Readback target differs from acknowledged command')
        if not np.allclose(tracking,abs(measured-command)*TICK,rtol=0,atol=1e-10) or np.max(tracking)>.18:raise ValueError('Tracking data differs from original checked state')
        if vision.get('anchor_id')!=41 or vision.get('cube_pose_frame')!='cube_center':raise ValueError('Wrist marker and cube-center board pose required')
        if vision.get('cube_frame_id')!=vision.get('anchor_frame_id'):raise ValueError('Cube and anchor must come from the same image')
        stamp=float(vision['pose_time'])
        if not np.isfinite(stamp) or stamp>now or now-stamp>.15:raise ValueError('Cube/anchor pose is stale')
        Rca=proper_rotation(vision['R_camera_anchor'],'camera anchor');Rcc=proper_rotation(vision['R_camera_cube'],'camera cube');tca=vector(vision['t_camera_anchor_m'],3,'camera anchor translation');tcc=vector(vision['t_camera_cube_m'],3,'camera cube translation')
        cube=self.twa+self.Rwa@(Rca.T@(tcc-tca));R=self.Rwa@Rca.T@Rcc
        return dict(sequence=int(sequence),sample_time=now,pose_time=stamp,frame_id='leap_sim_world',palm_point='policy_palm_position',measured_ticks=measured,commanded_ticks=command,palm_position=self.palm.copy(),cube_position=cube,cube_quaternion=matrix_quaternion(R),registration=dict(verified=self.verified,registration_id=self.registration_id),bus_checks_passed=True,observation_origin='simulator_fixture' if self.fixture else 'registered_bus_and_camera')
