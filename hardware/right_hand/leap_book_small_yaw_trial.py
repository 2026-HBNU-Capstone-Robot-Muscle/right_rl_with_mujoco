"""User-triggered small clockwise cube experiment. Default opens no devices."""
import sys
from book_small_yaw_plan import BookSmallYawPlan,ROOT
from book_small_yaw_gate import run_screen,validate_proof
from leap_book_cube_grasp_trial import BookGraspBus,Camera
from leap_thumb_prepare_current_empty import main


def require_offline_review():
    folder=ROOT/'right_hand_rotation_20261024/book20_current_review/small_yaw_fixture_v1'
    return validate_proof(BookSmallYawPlan(folder/'request.json'),folder/'screen.json')


BookSmallYawPlan.CAMERA_TYPE=Camera

if __name__=='__main__':
    if '--execute' in sys.argv:
        try:require_offline_review()
        except (OSError,ValueError,RuntimeError,KeyError,TypeError) as exc:
            print('OFFLINE REVIEW NOT READY: '+str(exc)+'. No camera or motor port opened.',flush=True)
            raise SystemExit(2)
    print('Leave book/base/fingers as they are. FIRST SPACE: EMPTY hand, cube OUT of camera.',flush=True)
    print('Only at PLACE CUBE: GREEN up, ORANGE toward camera, same place as the completed grasp.',flush=True)
    print('SECOND SPACE: hands clear; grasp -> 3s hold -> small clockwise experiment -> 2s hold.',flush=True)
    print('Only at REMOVE CUBE: remove cube, then THIRD SPACE = torque OFF. Q stops.',flush=True)
    raise SystemExit(main(BookSmallYawPlan,bus_type=BookGraspBus,screener=run_screen))
