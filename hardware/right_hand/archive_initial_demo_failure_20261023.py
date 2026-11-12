from pathlib import Path
root=Path(__file__).resolve().parent/'right_hand_rotation_20261023/slow_grasp_demo_fixture'
for source,target in (('native_screen.json','initial_release_failed_screen.json'),
                      ('fresh_off_request.json','initial_release_failed_request.json')):
    if (root/source).is_file() and not (root/target).exists():
        (root/target).write_bytes((root/source).read_bytes())
for source,target in (('native_screen.json','frontloaded_release_failed_screen.json'),
                      ('fresh_off_request.json','frontloaded_release_failed_request.json')):
    if (root/source).is_file() and not (root/target).exists():
        (root/target).write_bytes((root/source).read_bytes())
for source,target in (('native_screen.json','short_release_window_failed_screen.json'),
                      ('fresh_off_request.json','short_release_window_failed_request.json')):
    if (root/source).is_file() and not (root/target).exists():
        (root/target).write_bytes((root/source).read_bytes())
