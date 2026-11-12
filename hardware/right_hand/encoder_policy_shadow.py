"""Read-only encoder + camera policy diagnostic. No servo WRITE API is used.
Torque must stay OFF. Stored goals are telemetry, not commands from this script.
No simulated actuation, accumulated targets, or hardware-control approval.
"""
import datetime
import json
import sys
import time
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parent
PROJECT=ROOT.parent/'work/Leap_Test'
sys.path.insert(0,str(PROJECT/'deploy'))
from leap_hw import hw_to_sim, SIM2ARR
import limits
import obs_builder as ob
from policy_np import Policy
TICK=2*np.pi/4096


class ReadOnlyBus:
    def __init__(self,sdk):
        self.sdk=sdk
        self.port=sdk.PortHandler('COM7')
        self.packet=sdk.PacketHandler(2.0)

    def read(self,motor,addr,size):
        fn={1:self.packet.read1ByteTxRx,2:self.packet.read2ByteTxRx,4:self.packet.read4ByteTxRx}[size]
        value,comm,error=fn(self.port,motor,addr)
        if comm!=self.sdk.COMM_SUCCESS or error:
            raise RuntimeError(f'ID{motor} read {addr} failed: {comm}/{error}')
        return int(value)

    def open(self):
        if not self.port.openPort(): raise RuntimeError('Cannot open COM7; close other motor programs')
        if not self.port.setBaudRate(3000000): raise RuntimeError('Cannot set host baud')
        for i in range(1,17):
            model,comm,error=self.packet.ping(self.port,i)
            if comm!=self.sdk.COMM_SUCCESS or error or model!=1240:
                raise RuntimeError(f'ID{i} model/communication mismatch')
            for addr,size,expected in ((64,1,0),(70,1,0),(10,1,0),(11,1,5),(20,4,0)):
                if self.read(i,addr,size)!=expected:
                    raise RuntimeError(f'ID{i} unexpected register {addr}. No correction sent.')
        return self.snapshot()

    def snapshot(self):
        began=time.monotonic()
        fields=(('torque',64,1),('error',70,1),('goal',116,4),('position',132,4),
                ('voltage',144,2),('temperature',146,1))
        group=self.sdk.GroupSyncRead(self.port,self.packet,64,83)
        try:
            for i in range(1,17):
                if not group.addParam(i): raise RuntimeError(f'Cannot add ID{i} to read')
            comm=group.txRxPacket()
            if comm!=self.sdk.COMM_SUCCESS: raise RuntimeError(f'Encoder sync read failed: {comm}')
            state={}
            for name,addr,size in fields:
                values=[]
                for i in range(1,17):
                    if not group.isAvailable(i,addr,size): raise RuntimeError(f'ID{i}: missing fresh data')
                    values.append(int(group.getData(i,addr,size)))
                state[name]=np.array(values)
            validate_state(state)
            state['read_duration_ms']=(time.monotonic()-began)*1000
            return state
        finally:
            group.clearParam()

    def close(self):
        self.port.closePort()


def validate_state(state, *, expected_torque=0):
    if expected_torque not in (0,1): raise ValueError('Invalid expected torque state')
    for name in ('torque','error','goal','position','voltage','temperature'):
        v=np.asarray(state[name])
        if v.shape!=(16,) or not np.isfinite(v).all(): raise ValueError('Invalid encoder state')
    if np.any(state['torque']!=expected_torque) or np.any(state['error']):
        raise RuntimeError('Torque ON/OFF mismatch or motor error: stop, cut motor power. No OFF command is sent by this reader.')
    for name in ('position','goal'):
        if np.any(state[name]<0) or np.any(state[name]>4095): raise ValueError('Non single-turn encoder/goal')
    if np.any(state['voltage']<35) or np.any(state['voltage']>65) or np.any(state['temperature']>45):
        raise RuntimeError('Motor voltage/temperature outside diagnostic bounds')


def pose_rejection(pose, host_cycle_ms):
    """Shared preview/inference gate; None means this frame is eligible."""
    if not pose: return 'no_pose'
    if pose.get('n_cube_markers', 0) < 2: return 'cube_markers_below_2'
    if not np.isfinite(host_cycle_ms) or host_cycle_ms > 200: return 'frame_over_200ms'
    p = np.asarray(pose.get('palm_minus_cube'), float)
    q = np.asarray(pose.get('cube_quat_world'), float)
    reproj = float(pose.get('reproj', np.nan))
    if p.shape != (3,) or not np.isfinite(p).all(): return 'hand_cube_position_missing'
    if q.shape != (4,) or not np.isfinite(q).all(): return 'cube_orientation_missing'
    if not np.isfinite(reproj) or not 0 <= reproj <= 3: return 'reprojection_over_3px_or_invalid'
    if not np.isclose(np.linalg.norm(q), 1, atol=.01): return 'quaternion_not_unit'
    return None


class Shadow:
    def __init__(self,policy, *, expected_torque=0):
        self.policy=policy
        self.expected_torque=expected_torque
        self.reset()

    def reset(self):
        self.builder=ob.ObsBuilder()
        self.count=0
        self.last_skip_reason='history_warmup'

    def step(self,state,pose,host_cycle_ms):
        validate_state(state,expected_torque=self.expected_torque)
        reason = pose_rejection(pose, host_cycle_ms)
        if reason:
            self.reset(); self.last_skip_reason=reason; return None
        p=np.asarray(pose.get('palm_minus_cube'),float)
        q=np.asarray(pose.get('cube_quat_world'),float)
        reproj=float(pose.get('reproj',np.nan))
        actual=hw_to_sim(state['position']*TICK)
        stored_goal=hw_to_sim(state['goal']*TICK)
        obs=self.builder.build(actual,stored_goal,p,np.zeros(3),q,ob.FACE_GOAL_QUATS[0])
        self.count+=1
        if self.count<ob.HISTORY_LEN:
            self.last_skip_reason='history_warmup'; return None
        action=np.asarray(self.policy(obs),float)
        if obs.shape!=(108,) or action.shape!=(16,) or not np.isfinite(np.r_[obs,action]).all():
            raise ValueError('Nonfinite/invalid observation or action')
        if np.max(np.abs(action))>1.000001: raise ValueError('Action outside normalized range')
        # Never feed unsent actions into last_act, and never integrate a target.
        outside=np.flatnonzero((actual<limits.SAFE_LOWERS)|(actual>limits.SAFE_UPPERS))
        self.last_skip_reason=None
        return dict(q_measured_rad=actual.tolist(),q_stored_goal_rad=stored_goal.tolist(),
            observation=obs.tolist(),action=action.tolist(),unsent_delta_rad=(limits.ACTION_SCALE*action).tolist(),
            outside_policy_motor_ids=[int(SIM2ARR[i])+1 for i in outside],
            host_cycle_ms=float(host_cycle_ms),encoder_read_ms=float(state['read_duration_ms']),
            reprojection_px=reproj,palm_minus_cube_m=p.tolist(),cube_quat_world=q.tolist())


def main():
    report=dict(timestamp=datetime.datetime.now().astimezone().isoformat(),status='not_started',
        servo_write_commands_sent=0,torque_changes_requested=0,hardware_authorized=False,
        joint_source='live encoder + stored goal registers; torque OFF',
        last_action_source='zero: no policy action sent; not closed-loop validation',
        timing_note='Host capture/read/vision duration, NOT sensor exposure or synchronization guarantee',
        frames=0,samples=[],read_failures=0)
    bus=cap=cv=None
    try:
        import cv2 as cv
        import dynamixel_sdk as sdk
        from camera_720_check import open_hd
        from hand_registration_geometry import candidate
        sys.path.insert(0,str(PROJECT/'vision/rubik_center'))
        import cube_board as cb
        import track_cube as tc
        registration=candidate()
        tc.PALM_IN_MARKER=np.array(registration['grasp_site_in_marker_m_candidate'])
        tc.HAND_TO_WORLD=np.array(registration['hand_to_world_candidate'])
        report['registration']=registration
        report['registration_depth_verified']=False
        with np.load(ROOT/'camera1_720.npz') as cal:
            if (int(cal['height']),int(cal['width']))!=(720,1280): raise ValueError('Wrong camera calibration size')
            K,dist=cal['K'].copy(),cal['dist'].copy()
        rotations=tuple(json.loads((ROOT/'cube_face_rot_result.json').read_text())['face_rot'])
        points,ids=cb.object_points(cube_mm=54.5,marker_mm=10.6,layout='centers',face_rot=rotations)
        dictionary,_,detector=tc.build(marker_mm=10.6,grid=1)
        board=cv.aruco.Board(points,dictionary,ids)
        shadow=Shadow(Policy(PROJECT/'policy/policy_np.npz'))
        bus=ReadOnlyBus(sdk)
        state=bus.open()
        report['initial_positions_raw']=state['position'].tolist()
        cap,capture,_=open_hd(cv,1,require_mjpg=True)
        report['capture']=capture
        report['face_rot']=rotations
        title='ENCODER POLICY SHADOW - READ ONLY'
        cv.namedWindow(title,cv.WINDOW_NORMAL); cv.resizeWindow(title,1100,620)
        print('READ ONLY: live encoder + webcam. PING/READ only; NO torque/goal/settings writes.',flush=True)
        print('Torque must remain OFF. Show ID40 and 2 cube markers. SPACE: 10s, Q: quit.',flush=True)
        print('Unsent actions / unverified registration: this does NOT approve policy motion.',flush=True)
        started=None
        last_frame=time.monotonic()
        while True:
            t0=time.monotonic()
            if started is not None and t0-started>=10:
                report['status']='complete_readonly_not_motion_approval'; break
            ok,frame=cap.read()
            state=bus.snapshot()
            if not ok or frame is None:
                report['read_failures']+=1; shadow.reset()
                if time.monotonic()-last_frame>5: raise RuntimeError('No camera frame for 5 seconds')
                if cv.waitKey(1)&255 in (ord('q'),27): report['status']='user_stopped'; break
                continue
            last_frame=time.monotonic()
            if frame.shape[:2]!=(720,1280): raise RuntimeError('Camera resolution changed')
            pose=tc.step(frame,board,detector,dictionary,K,dist)
            snapshot=shadow.step(state,pose,(time.monotonic()-t0)*1000)
            actual=hw_to_sim(state['position']*TICK)
            outside=np.flatnonzero((actual<limits.SAFE_LOWERS)|(actual>limits.SAFE_UPPERS))
            lines=['READ ONLY | ALL TORQUE OFF | LIVE ENCODERS',
                f'Cube markers: {pose.get("n_cube_markers",0) if pose else 0} | inference: {"YES - NOT SENT" if snapshot else "WAIT / SKIP"}',
                'Outside policy IDs: '+str([int(SIM2ARR[i])+1 for i in outside]),
                'Unverified registration | SPACE 10s | Q quit']
            if snapshot:
                lines.append(f'Max unsent delta: {np.max(np.abs(snapshot["unsent_delta_rad"])):.3f}rad')
            if started is not None:
                report['frames']+=1
                if snapshot: report['samples'].append(dict(snapshot,time_s=time.monotonic()-started))
                lines.append(f'{max(0,10-(time.monotonic()-started)):.1f}s left | samples {len(report["samples"])}')
            for i,line in enumerate(lines):
                cv.putText(frame,line,(10,25+25*i),cv.FONT_HERSHEY_SIMPLEX,.55,(0,255,255),2)
            cv.imshow(title,frame)
            key=cv.waitKey(1)&255
            if key in (ord('q'),27): report['status']='user_stopped'; break
            if key==32 and started is None:
                shadow.reset(); started=time.monotonic()
                print('Measuring 10 seconds; no servo commands.',flush=True)
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped_error',error=str(exc) or 'Interrupted')
        print('STOP: '+report['error'],flush=True)
    finally:
        if bus is not None:
            try:
                final=bus.snapshot()
                report['all_torque_off_observed_at_end']=True
                report['final_positions_raw']=final['position'].tolist()
            except Exception as exc:
                report['all_torque_off_observed_at_end']=False
                report['final_read_error']=str(exc)
                print('Motor state unverified: cut motor power. This program sends no OFF command.',flush=True)
            bus.close()
        if cap is not None: cap.release()
        if cv is not None: cv.destroyAllWindows()
        n=len(report['samples']); frames=report['frames']
        report['inference_percent']=100*n/frames if frames else None
        report['all_joints_in_policy_range_samples']=sum(not s['outside_policy_motor_ids'] for s in report['samples'])
        tag=datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        for name in (f'encoder_policy_shadow_{tag}.json','encoder_policy_shadow_result.json'):
            path=ROOT/name; path.write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(f'Saved: {path}\nInference samples {n}/{frames}; servo writes: 0. NOT motion approval.',flush=True)
    return 0 if report['status']=='complete_readonly_not_motion_approval' else 1


if __name__=='__main__': raise SystemExit(main())
