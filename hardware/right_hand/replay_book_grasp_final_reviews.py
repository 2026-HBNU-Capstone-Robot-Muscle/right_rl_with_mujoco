"""Two full native replays, preserving all prior failed/successful artifacts."""
from pathlib import Path
from screen_book_grasp_trial import main

root=Path(__file__).resolve().parent
folders=(root/'right_hand_rotation_20261024/book20_current_review/grasp_route_fixture_v2',
    root/'book_settled_off_readonly_20261024_171618_149355')
codes=[]
for folder in folders:
    print('Full native replay:',folder.name,flush=True)
    codes.append(main(folder/'request.json',folder/'screen_measured_release.json'))
raise SystemExit(0 if codes==[0,0] else 2)
