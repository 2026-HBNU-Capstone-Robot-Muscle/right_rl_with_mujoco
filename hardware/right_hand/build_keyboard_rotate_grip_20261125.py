"""Bind the keyboard runner to the current checked horizontal/vertical trials."""
import hashlib
import json
from leap_keyboard_rotate_grip_20261125 import ROOT, BASE, KeyboardRotationSequence


def main():
    names = ['leap_keyboard_rotate_grip_20261125.py', 'leap_keyboard_rotate_grip.cmd',
             'build_keyboard_rotate_grip_20261125.py', 'test_keyboard_rotate_grip_20261125.py']
    trial = 'right_hand_rotation_20261125/rotation_3x_with_recovery/'
    names += [trial+axis+'/'+name for axis in ('horizontal', 'vertical')
              for name in ('manifest.json', 'candidate.npz')]
    sequence = KeyboardRotationSequence({}, verify=False)
    sequence.rows(sequence.start)
    manifest = dict(kind='keyboard_AB_current_experimental_rotations',
        source_hashes={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in names},
        **sequence.metadata())
    BASE.mkdir(parents=True, exist_ok=True)
    (BASE/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print('Keyboard A/B runner bound to current checked axis candidates; no devices opened.')


if __name__ == '__main__':
    main()
