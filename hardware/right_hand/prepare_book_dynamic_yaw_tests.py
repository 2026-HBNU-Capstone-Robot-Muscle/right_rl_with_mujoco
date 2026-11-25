"""Adapt the existing meaningful PLACE/fault suite to the new frozen route."""
from pathlib import Path
root=Path(__file__).resolve().parent
source=(root/'test_book_small_yaw.py').read_text(encoding='utf-8')
changes={
    'import book_small_yaw_plan as p':'import book_dynamic_yaw_plan as p\nfrom book_small_yaw_plan import BASELINE,AFTER',
    'import leap_book_small_yaw_trial as driver':'import leap_book_dynamic_yaw_trial as driver',
    'from book_small_yaw_capture import BookSmallYawCapture':'from book_dynamic_yaw_capture import BookDynamicYawCapture as BookSmallYawCapture',
    'small_yaw_fixture_v1':'dynamic_yaw_fixture_v2',
    'p.BookSmallYawPlan':'p.BookDynamicYawPlan',
    'p.BASELINE':'BASELINE','p.AFTER':'AFTER',
    '[60,90,40]':'[60,126,40]',
    "r['goal_ticks'][-1][7]+=1":"r['goal_ticks'][-1][11]+=1",
    'len(bus.pending),469':'len(bus.pending),505',
    "self.assertTrue(report['cube_removal_confirmed'] and report['rotation_command_path_completed'])":
        "self.assertTrue(report['cube_removal_confirmed'] and report['rotation_command_path_completed'])\n"
        "        self.assertTrue(report['cube_removal_operator_confirmed'])\n"
        "        self.assertFalse(report['cube_removal_physically_verified'])\n"
        "        self.assertIsNone(report['cube_present'])",
}
for old,new in changes.items():
    if old not in source:raise ValueError('Test adaptation source changed: '+old)
    source=source.replace(old,new)
target=root/'test_book_dynamic_yaw.py'
target.write_text(source,encoding='utf-8')
print('Wrote the revised route/candidate/proof/PLACE/fault/removal-evidence tests.')
