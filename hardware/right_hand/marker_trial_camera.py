"""Raw recording at measured exposure -7; no marker gate during motor motion."""
import functools
import json
from pathlib import Path
import time

from empty_trial_camera import EmptyTrialCamera
from fixed_exposure_camera import fixed_exposure_worker
from camera_frame_signal import CameraImageUnavailable
from empty_start_marker_check import make_empty_detector,inspect_empty_frame,EmptyStartEvidence

EMPTY_SCREEN_FRAMES=10
EMPTY_SCREEN_PHASES=frozenset(('preview','preflight','stable_off_readings',
                              'screen_current_pose','countdown'))


class MarkerTrialCamera(EmptyTrialCamera):
    EXPOSURE=-7.
    EMPTY_ONLY=False
    CONFIRM_PROPERTY_READS=False
    PRESERVE_EXPOSURE=False

    def __init__(self,folder,report):
        self.empty_detector=make_empty_detector()
        self.empty_evidence=EmptyStartEvidence()
        self.empty_checked_stamp=None
        self.empty_streak=0
        self.empty_quality=None
        self.restoration_path=Path(folder).parent/'camera_exposure_worker.json'
        worker=None
        if not self.PRESERVE_EXPOSURE:
            worker=functools.partial(fixed_exposure_worker,
                restoration_path=str(self.restoration_path),display_mode='record',exposure=self.EXPOSURE)
            if self.CONFIRM_PROPERTY_READS:
                worker=functools.partial(worker,confirm_property_reads=True)
        report['camera_requested_exposure']=None if self.PRESERVE_EXPOSURE else self.EXPOSURE
        report['camera_exposure_control']='unchanged' if self.PRESERVE_EXPOSURE else 'fixed_readback_verified'
        report['marker_visibility_is_motion_gate']=False
        report['empty_start_marker_screen_is_empty_hand_proof']=False
        super().__init__(folder,report,worker=worker)
        print('FIRST SPACE: hand must be EMPTY. Remove the cube from the hand AND camera view.',flush=True)
        print('Stable exact cube-marker evidence blocks EMPTY preparation. '+
              ('Cube stays OUT for this entire trial.' if self.EMPTY_ONLY else 'Cube only at PLACE CUBE.'),flush=True)

    def check(self):
        # Decode only before configuration/enable, never on the motor health
        # loop. A missing marker cannot certify that the physical hand is empty.
        phase=self.report.get('phase','preview')
        if self.report.get('enabled_at') is None and phase in EMPTY_SCREEN_PHASES:
            with self.lock:packet=self.latest
            if packet is not None:
                stamp,frame=packet
                if 0<=time.monotonic()-stamp<=.3 and stamp!=self.empty_checked_stamp:
                    candidates=inspect_empty_frame(frame,self.empty_detector)
                    quality=self.empty_evidence.observe(stamp,candidates)
                    self.empty_streak=quality['consecutive_frames_without_decoded_cube']
                    self.empty_checked_stamp=stamp
                    self.empty_quality=dict(quality,required_frames=EMPTY_SCREEN_FRAMES)
                    self.report['empty_start_marker_screen']=self.empty_quality.copy()
                quality=self.empty_quality
                if quality is not None:
                    cube_ids=quality['decoded_cube_ids']
                    self.report['camera_instruction']=(
                        'CUBE CONFIRMED; remove before EMPTY preparation' if quality['confirmed_cube_ids'] else
                        'Checking possible marker; keep hand EMPTY' if cube_ids else
                        'EMPTY ONLY; cube stays OUT' if self.EMPTY_ONLY else
                        'EMPTY HAND for FIRST SPACE; cube only after PLACE CUBE')
                    if quality['confirmed_cube_ids'] and phase!='preview':
                        # Preserve the exact raw frame that caused the stop;
                        # JPEG replay can otherwise change a marginal decode.
                        ok,data=self.cv.imencode('.png',frame)
                        if ok:
                            destination=self.folder.parent/'empty_start_blocked.png'
                            destination.write_bytes(data.tobytes())
                            self.report['empty_start_blocked_frame']=dict(image=destination.name,
                                camera_read_monotonic=stamp,quality=quality)
                        raise CameraImageUnavailable('Cube marker confirmed across fresh frames before EMPTY preparation. Remove cube; no motion started.')
        return super().check()

    def require_fresh(self):
        super().require_fresh()
        quality=self.empty_quality
        if quality is None or not 0<=time.monotonic()-quality['camera_read_monotonic']<=.3:
            raise CameraImageUnavailable('Wait for a fresh EMPTY-start marker check, then press SPACE again. No motor writes.')
        if quality is not None and quality['decoded_cube_ids']:
            raise CameraImageUnavailable('Possible cube marker: keep cube OUT of hand AND camera view, then press SPACE again. No motor writes.')
        if quality['consecutive_frames_without_decoded_cube']<EMPTY_SCREEN_FRAMES:
            raise CameraImageUnavailable('Keep the hand EMPTY for 10 fresh camera frames, then press SPACE again. No motor writes.')
        self.report['empty_start_marker_screen_accepted']=True

    def finish(self):
        result=super().finish()
        if self.PRESERVE_EXPOSURE:
            # No exposure GET/SET session exists in the default frame worker.
            # Do not mislabel an unknown readback as verified restoration.
            self.report['camera_exposure_settings_modified']=False
            self.report['camera_exposure_restoration_required']=False
            self.report['camera_exposure_restoration_verified']=None
            result['exposure_settings_modified']=False
            result['exposure_restoration_required']=False
            result['exposure_restoration_verified']=None
            return result
        worker=None
        if self.restoration_path.is_file():
            worker=json.loads(self.restoration_path.read_bytes())
        restored=bool(worker and worker.get('restoration',{}).get('restored_original_readbacks'))
        self.report['camera_exposure_worker']=worker
        self.report['camera_exposure_restoration_verified']=restored
        result['exposure_restoration_verified']=restored
        if not restored:
            result['status']='incomplete'
            result['exposure_restoration_note']='No verified restoration readbacks; motor OFF status is recorded separately.'
        return result
