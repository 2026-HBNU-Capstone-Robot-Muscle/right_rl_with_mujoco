"""Supervised prefix-only support photo; no loaded quarter-turn targets."""
import argparse,datetime,hashlib,json,time
from pathlib import Path
import numpy as np
from loaded_quarter_trial_plan_20261025 import LoadedQuarterTrialPlan,ROOT,TURN_PHASE,motion_goals
from leap_loaded_quarter_trial_20261025 import LoadedQuarterBus,Camera
from support_photo_path_plan_20261025 import SupportPhotoPathPlan,run_screen,FIXTURE,validate_proof
from support_photo_camera_20261025 import SupportPhotoCamera
from loaded_quarter_trial_protocol_20261025 import LoadedQuarterCapture
from quarter_current_runtime_20261025 import prepare_and_run
from leap_thumb_prefix_empty import record_off_endpoint
import hand_motor_mapping as mapping
REVIEW=ROOT/'right_hand_rotation_20261024/rigid_contact_review_20261025/photo_mode_review.json'
Camera=SupportPhotoCamera
class SupportPhotoBus(LoadedQuarterBus):
    def approve(self,plan,proof):
        validate_proof(plan,proof)
        return super().approve(plan,proof)
LoadedQuarterBus=SupportPhotoBus
class SupportPhotoComplete(Exception):
    """Expected end of the reviewed EMPTY prefix and PLACE/REMOVE checkpoints."""
class SupportPhotoCapture(LoadedQuarterCapture):
    def place(self,bus,command,report,guard):
        self.guard=guard;observe=self._observer(bus,command,report,guard)
        state,photo=self._photo('empty_support_reference.jpg','prepared_empty_reference',observe)
        report.update(diagnostic_mode='quarter_support_photo_only',prepared_empty_photo=photo,
            prepared_empty_ticks=state['position'].tolist(),cube_rotation_requested=False,rotation_included=False)
        report['camera_instruction']='SIDE SUPPORT PHOTO: show cube bottom edge and actual support; no rotation follows.'
        print('PLACE CUBE as last time: GREEN up, ORANGE toward WRIST/ID41. No rotation will run.',flush=True)
        print('Aim camera from the SIDE: show the cube BOTTOM EDGE and the fingers directly beneath it.',flush=True)
        print('Do not move robot base or fingers for the camera. Cube must rest without your hand.',flush=True)
        print('Hands clear. SECOND SPACE in TERMINAL saves the support photo only. Q stops.',flush=True)
        self._space('place_cube',observe)
        state,photo=self._photo('cube_support_side.jpg','place_cube',observe)
        report.update(placement_photo=photo,placement_position_ticks=state['position'].tolist(),
            placement_goal_ticks=np.asarray(command).tolist(),cube_placement_confirmed=True,cube_present=True,
            registration_verified=False,loaded_goal_count=0)
        report['camera_instruction']='REMOVE CUBE, then THIRD SPACE for OFF; no loaded goals.'
        print('Photo saved. REMOVE CUBE, then THIRD SPACE in TERMINAL confirms removal and turns torque OFF.',flush=True)
        state=self._space('remove_cube',observe)
        report.update(cube_removal_confirmed=True,cube_present=False,physical_flip_verified=False,
            final_on_state={k:v.tolist() for k,v in state.items()},diagnostic_completion_requested=True)
        raise SupportPhotoComplete()
    def finish(self,*args):raise RuntimeError('Support-photo mode must never reach the loaded path or its end callback')
class SupportPhotoPlan(SupportPhotoPathPlan):
    TITLE='SIDE SUPPORT PHOTO ONLY: EMPTY return -> PLACE/photo -> REMOVE -> OFF. NO loaded motion.'
    FOLDER_PREFIX='quarter_support_photo'
    STATUS='quarter_support_photo_complete'
    MODEL_LIMITS_DESCRIPTION='user ID4 lower -2deg; all other joint limits unchanged'
    CAMERA_TYPE=Camera
    CAPTURE_PROTOCOL_TYPE=SupportPhotoCapture
def require_offline_review():
    return validate_proof(SupportPhotoPlan(FIXTURE/'request.json'),FIXTURE/'screen.json')
def require_mode_review():
    r=json.loads(REVIEW.read_bytes());this=Path(__file__).resolve()
    if (r.get('script_sha256')!=hashlib.sha256(this.read_bytes()).hexdigest()
        or r.get('device_free_tests_passed')!=7 or r.get('loaded_goals_sent_in_mock')!=0
        or r.get('off_and_settings_restored_in_mock') is not True
        or r.get('off_camera_tests_passed')!=7
        or r.get('off_camera_adjustment_sha256')!=hashlib.sha256((ROOT/'support_photo_camera_20261025.py').read_bytes()).hexdigest()
        or r.get('native_full_route_proof_sha256')!=hashlib.sha256((FIXTURE/'screen.json').read_bytes()).hexdigest()):
        raise RuntimeError('Support photo mode review mismatch; no devices opened')
    require_offline_review()
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--execute',action='store_true');args=parser.parse_args()
    print(SupportPhotoPlan.TITLE,flush=True)
    if not args.execute:print('CHECK ONLY: no camera, motor port, or simulator process opened.');return 0
    require_mode_review()
    import msvcrt,dynamixel_sdk as sdk
    from prepared_placement_capture import TerminalKeys
    from camera_frame_signal import CameraImageUnavailable
    stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f');folder=ROOT/f'quarter_support_photo_{stamp}';folder.mkdir()
    report=dict(timestamp=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=9))).isoformat(),status='not_started',phase='preview',
        joint_mapping=mapping.metadata(),policy_used=False,physical_flip_verified=False,all_torque_off_verified=False,settings_restored=False,
        hardware_authorized=False,recording_requested=False,diagnostic_mode='quarter_support_photo_only',loaded_goal_count=0,
        mode_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    bus=camera=None;opened=False
    try:
        camera=Camera(folder/'camera',report)
        print('EMPTY hand. Aim webcam lower, from the side, to show the future cube bottom and support.',flush=True)
        print('FIRST SPACE in TERMINAL starts the reviewed EMPTY preparation. Q cancels.',flush=True)
        while True:
            camera.check()
            if msvcrt.kbhit():
                key=msvcrt.getwch()
                if key in ('q','Q','\x1b','\x03'):raise KeyboardInterrupt('User stop')
                if key==' ':
                    try:camera.require_fresh()
                    except CameraImageUnavailable as exc:print(str(exc),flush=True);continue
                    break
            time.sleep(.01)
        report.update(hardware_authorized=True,recording_requested=True,phase='preflight')
        keys=TerminalKeys();camera.off_wait_cancel=keys.poll
        def cancel():camera.check();keys.poll()
        bus=LoadedQuarterBus(sdk,report);bus.planner=SupportPhotoPlan.validate_admission;bus.cancel_callback=cancel
        if not bus.port.openPort():raise RuntimeError('Cannot open COM7')
        opened=True
        if not bus.port.setBaudRate(3000000):raise RuntimeError('Cannot set baud')
        bus.preflight();protocol=SupportPhotoCapture(camera,keys,cancel=cancel)
        try:
            prepare_and_run(bus,folder,report,cancel=cancel,plan_type=SupportPhotoPlan,screener=run_screen,
                prepared_callback=protocol.finish,phase_callback=protocol.place)
        except SupportPhotoComplete:
            if (not report.get('cube_removal_confirmed') or not report.get('all_torque_off_verified') or not report.get('settings_restored')
                or any(s['phase']==TURN_PHASE for s in report.get('samples',[]))
                or report.get('last_issued_command')!=motion_goals()[0].tolist()):
                raise RuntimeError('Support photo shutdown/EMPTY-prefix evidence incomplete')
            report.update(status=SupportPhotoPlan.STATUS,loaded_goal_count=0,stopped_before_loaded_motion=True)
            print('Support photo complete. No loaded motion. Torque OFF verified; settings restored.',flush=True)
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped',error=str(exc) or 'Interrupted',failed_phase=report.get('phase'));print('STOP:',report['error'],flush=True)
    finally:
        if bus is not None and opened:
            if report.get('settings_restored') and report.get('all_torque_off_verified'):record_off_endpoint(bus,report)
            try:report['all_torque_off_verified']=bool(np.all(bus.block(64,1,[('torque',64,1)])['torque']==0))
            except Exception as exc:report.update(all_torque_off_verified=False,final_check_error=str(exc))
            if not report['all_torque_off_verified']:print('CUT MOTOR POWER: torque OFF unverified.',flush=True)
            bus.port.closePort()
        if camera is not None:
            try:report['camera_recording']=camera.finish()
            except Exception as exc:report['camera_save_error']=str(exc)
        payload=json.dumps(report,ensure_ascii=False,indent=2);(folder/'result.json').write_text(payload,encoding='utf-8')
        (ROOT/'leap_quarter_support_photo_result.json').write_text(payload,encoding='utf-8');print('Saved:',folder,flush=True)
    return 0 if report['status']==SupportPhotoPlan.STATUS and report['all_torque_off_verified'] and report['settings_restored'] else 1
if __name__=='__main__':raise SystemExit(main())
