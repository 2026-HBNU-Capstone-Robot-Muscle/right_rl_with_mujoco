"""Manual joint teaching after verified torque OFF. Only PING/READ are supplied.

Corrected encoders are evidence, never replacement goals or permission to
re-enable. OFF sag, hand support and uncalibrated geometry remain explicit.
"""
import datetime as dt,hashlib,json,time
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parent
FIELDS=[('torque',64,1),('error',70,1),('d_gain',80,2),('i_gain',82,2),('p_gain',84,2),('watchdog',98,1),('current_cap',102,2),('acceleration',108,4),('velocity',112,4),('goal',116,4),('current',126,2),('position',132,4),('voltage',144,2),('temperature',146,1)]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def require_completed_off_motion(report):
    if report.get('status')!='manual_grasp_reached_torque_off' or not all(report.get(k) is True for k in ('all_torque_off_verified','settings_restored','cube_support_confirmed_before_off','cube_placement_confirmed')):raise RuntimeError('Motion did not finish with verified OFF/restoration/support; manual teaching not started')
    if report.get('automatic_open') is not False or report.get('manual_joint_correction_allowed') is not False:raise RuntimeError('Wrong teaching handoff scope')
def validate_off_snapshot(state):
    for key in ('torque','error','watchdog','position','voltage','temperature'):
        a=np.asarray(state[key])
        if a.shape!=(16,) or not np.isfinite(a).all() or np.any(a!=np.rint(a)):raise RuntimeError('Incomplete OFF teaching telemetry')
    if any(np.any(state[k]) for k in ('torque','error','watchdog')):raise RuntimeError('Torque/error/watchdog is not clear. Stop manual correction and cut motor power.')
    if min(state['voltage'])<38 or max(state['voltage'])>65 or max(state['temperature'])>45:raise RuntimeError('OFF teaching health outside checked range')
    # Native angle ranges are deliberately not used as an encoder clamp.
    # A freely taught pose outside them is recorded and requires model review.
    return np.asarray(state['position'],dtype=np.int64)
def install_readonly_guard(port,report):
    original=port.writePort
    def readonly(data):
        if len(data)<8 or data[7] not in (1,2,0x8a):raise RuntimeError('Non-READ motor instruction blocked during OFF teaching')
        if data[7]==0x8a:
            ids=list(data[12:-2]);groups=[list(range(n,n+4)) for n in (1,5,9,13)]
            if list(data[8:12])!=[64,0,83,0] or ids not in groups:raise RuntimeError('Fast READ outside documented teaching scope')
        key=str(int(data[7]));counts=report.setdefault('instruction_counts',{});counts[key]=counts.get(key,0)+1
        return original(data)
    port.writePort=readonly
class TeachingReadOnlyBus:
    def __init__(self,report,cancel):self.report=report;self.cancel_callback=cancel;self._read_recovery_times=[];self.port=None
    def open(self):
        import dynamixel_sdk as sdk
        from serial.tools.list_ports import comports
        from horizontal_serial_link_20261028 import metadata
        from calibrated_precise_clock_20261028 import install
        cfg=metadata();devices=[p for p in comports() if p.device==cfg['port'] and p.vid==0x0403 and p.pid==0x6014 and p.serial_number=='FTBENPSAA']
        if len(devices)!=1:raise RuntimeError('Verified robot USB adapter missing or ambiguous')
        self.sdk=sdk;self.packet=sdk.PacketHandler(2.);self.port=sdk.PortHandler(devices[0].device)
        if not self.port.openPort() or not self.port.setBaudRate(cfg['baud']):raise RuntimeError('Cannot open verified1Mbps READ-only link')
        install_readonly_guard(self.port,self.report);install(self)
        for motor in range(1,17):
            model,comm,error=self.packet.ping(self.port,motor)
            if comm or error or model!=1240:raise RuntimeError(f'ID{motor} PING failed')
            fw,comm,error=self.packet.read1ByteTxRx(self.port,motor,6)
            if comm or error or fw<46:raise RuntimeError('Fast READ firmware unsupported')
        self.report['serial_link_policy']=cfg
    def snapshot(self):
        from fast_health_read4_20261029 import block_once
        from combined_quiet_reserve_20261029 import receive
        self._horizontal_sample_deadline=time.perf_counter()+.2
        state=receive(self,lambda:block_once(self,64,83,FIELDS),'OFF teaching READ',dict(kind='sync_read',address=64,length=83))
        state['position']=np.where(state['position']>=(1<<31),state['position']-(1<<32),state['position'])
        validate_off_snapshot(state);return state
    def close(self):
        if self.port is not None and getattr(self.port,'ser',None) is not None:self.port.closePort()
def inspector(read_snapshot):
    def inspect(row):
        state=read_snapshot();position=validate_off_snapshot(state);row['raw_snapshot']={k:np.asarray(v).tolist() for k,v in state.items()}
        row['motors']=[dict(id=i+1,torque_enabled=int(state['torque'][i]),torque_enabled_end=int(state['torque'][i]),hardware_error=int(state['error'][i]),bus_watchdog_raw=int(state['watchdog'][i]),position_ticks_signed=int(position[i]),voltage_v=float(state['voltage'][i])/10,temperature_c=int(state['temperature'][i])) for i in range(16)];row['status']='read_complete_not_motion_approval'
    return inspect
def capture_off_pair(read_snapshot,camera,report,save_image,**kwargs):
    from leap_pose_pair_readonly import capture_pair
    return capture_pair(inspector(read_snapshot),camera,report,save_image,**kwargs)
def run(source_motion):
    source_motion=Path(source_motion).resolve();source_motion.relative_to(R);source=json.loads(source_motion.read_bytes());require_completed_off_motion(source)
    import cv2
    from camera_stream import CameraStream
    from camera_frame_signal import frame_signal
    from prepared_placement_capture import TerminalKeys
    from hand_motor_mapping import metadata as mapping_metadata
    from loaded_quarter_reentry_20261025 import limits
    out=R/('manual_grasp_teach_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));out.mkdir();keys=TerminalKeys();camera=bus=None;accepted=False
    report=dict(status='started',phase='off_manual_joint_teaching',source_motion_result=source_motion.relative_to(R).as_posix(),source_motion_sha256=sha(source_motion),joint_mapping=mapping_metadata(),servo_writes=0,torque_changes=0,automatic_reenable=False,corrected_pose_motion_authorized=False,manual_cube_support_may_be_required=True,geometry_calibration_verified=False,hardware_execution_allowed=False,attempts=[])
    try:
        def cancel():
            keys.poll()
            if camera is not None:camera.check_preview('TORQUE OFF. Reshape joints gently; support cube. SPACE saves. No re-enable.')
        bus=TeachingReadOnlyBus(report,cancel);bus.open();baseline=[bus.snapshot(),bus.snapshot()];report['before_manual_OFF_snapshots']=[{k:v.tolist() for k,v in s.items()} for s in baseline];report['last_powered_position_ticks']=source['final_on_state']['position'];report['last_powered_goal_ticks']=source['final_on_state']['goal']
        camera=CameraStream(report,preview_title='FINGER TEACH | TORQUE OFF');keys.arm();began=last_read=time.monotonic();attempt=0
        print('\n[토크OFF 보정] 16개 모터의 토크OFF를 다시 확인했습니다.',flush=True)
        print('5V 전원은 켜둡니다. 모터 힘은 꺼져 있어요. 큐브를 받치고 원하는 손가락 각도만 천천히 바꾸세요.',flush=True)
        print('보정한 모양을 유지한 채 이 터미널에서 네 번째 SPACE를 누르면 사진과 관절값을 저장합니다.',flush=True)
        print('저장 뒤에도 토크를 다시 켜지 않습니다. 손가락을 억지로 꺾거나 모터 프로그램을 따로 켜지 마세요. Q 종료.',flush=True)
        while time.monotonic()-began<180:
            cancel();now=time.monotonic()
            if now-last_read>=.1:bus.snapshot();last_read=time.monotonic()
            if keys.consume_space():
                attempt+=1;folder=out/f'capture_{attempt:02d}';folder.mkdir();record={};report['attempts'].append(record)
                def save(frame):
                    signal=frame_signal(frame)
                    if not signal['usable']:raise RuntimeError('Blank camera image; corrected pose not accepted')
                    ok,encoded=cv2.imencode('.png',frame)
                    if not ok:raise RuntimeError('PNG encode failed')
                    (folder/'paired_photo.png').write_bytes(encoded.tobytes());record['image_signal']=signal
                accepted=capture_off_pair(bus.snapshot,camera,record,save)
                (folder/'result.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
                if accepted:
                    corrected=np.array([r['position_ticks_signed'] for r in record['snapshots'][-1]['motors']]);off=np.asarray(baseline[-1]['position']);on=np.asarray(source['final_on_state']['position']);lo,hi=limits();report.update(status='manual_joint_pose_and_photo_saved',corrected_pose_folder=folder.relative_to(R).as_posix(),corrected_position_ticks=corrected.tolist(),change_from_initial_OFF_ticks=(corrected-off).tolist(),change_from_last_powered_ticks=(corrected-on).tolist(),outside_configured_model_range_ids=[i+1 for i in range(16) if corrected[i]<lo[i] or corrected[i]>hi[i]],manual_and_passive_changes_not_causally_separated=True,all_observed_torque_off=True);break
                print('촬영 전후 관절값이3틱보다 많이 달라졌어요. 모양을 잠깐 유지하고 SPACE를 다시 누르세요. 토크는 계속OFF입니다.',flush=True);keys.arm()
            time.sleep(.02)
        else:raise RuntimeError('OFF teaching180s timeout; no motor command sent')
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc) or 'User stop')
    finally:
        if bus is not None:bus.close()
        if camera is not None:camera.release()
        payload=json.dumps(report,indent=2,ensure_ascii=False)+'\n';(out/'result.json').write_text(payload,encoding='utf-8');(R/'leap_manual_grasp_teach_result.json').write_text(payload,encoding='utf-8')
    print('Saved:',out,flush=True)
    if accepted:print('보정 자세 저장 완료. 모터 힘은 계속OFF입니다. 토크를 직접 켜지 말고 기록 검토를 기다려주세요.',flush=True)
    else:print('보정 저장 미완료:',report.get('error'),flush=True)
    return 0 if report['status']=='manual_joint_pose_and_photo_saved' else 1
