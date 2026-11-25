"""Supervised revised small turn; no devices opened in default check mode."""
import sys
from book_dynamic_yaw_plan import ROOT,BookDynamicYawPlan
from book_dynamic_yaw_gate import run_screen,validate_proof
from leap_book_cube_grasp_trial import BookGraspBus,Camera
from leap_thumb_prepare_current_empty import main


def require_offline_review():
    folder=ROOT/'right_hand_rotation_20261024/book20_current_review/dynamic_yaw_fixture_v2'
    return validate_proof(BookDynamicYawPlan(folder/'request.json'),folder/'screen.json')


BookDynamicYawPlan.CAMERA_TYPE=Camera

if __name__=='__main__':
    if '--execute' in sys.argv:
        try:require_offline_review()
        except (OSError,ValueError,RuntimeError,KeyError,TypeError) as exc:
            print('OFFLINE REVIEW NOT READY: '+str(exc)+'. No camera or motor port opened.',flush=True)
            raise SystemExit(2)
    print('REVISED v2. FIRST SPACE: EMPTY hand. Leave book/base/fingers as they are.',flush=True)
    print('PLACE CUBE: GREEN up, ORANGE toward camera; same place as before. Hands clear, SECOND SPACE.',flush=True)
    print('Grasp -> 3s hold -> revised small clockwise experiment -> 2s hold. Q stops.',flush=True)
    print('REMOVE CUBE FIRST. Only then THIRD SPACE switches torque OFF.',flush=True)
    raise SystemExit(main(BookDynamicYawPlan,bus_type=BookGraspBus,screener=run_screen))
