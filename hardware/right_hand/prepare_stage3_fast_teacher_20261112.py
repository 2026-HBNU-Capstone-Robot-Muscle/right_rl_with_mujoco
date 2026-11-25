"""Resume this task's model search; actual executed states still check every1ms."""
import hashlib
import json
import os
from pathlib import Path
import signal
import time

ROOT = Path(__file__).resolve().parent
OLD = ROOT / 'stage3_relative_teacher_20261112.py'
OUT = ROOT / 'right_hand_rotation_20261112/stage3_preparation'
NEW = ROOT / 'stage3_fast_teacher_20261112.py'


def main():
    if os.name != 'posix' or NEW.exists():
        raise RuntimeError('Linux task-specific one-time continuation required')
    matches = []
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            if str(OLD).encode() in path.read_bytes().split(b'\0'):
                matches.append(int(path.parent.name))
        except (OSError, PermissionError):
            continue
    if len(matches) > 1:
        raise RuntimeError('Ambiguous simulation process')
    for pid in matches:
        os.kill(pid, signal.SIGINT)
    for _ in range(30):
        if all(not Path(f'/proc/{pid}').exists() for pid in matches): break
        time.sleep(.1)
    else:
        raise RuntimeError('Model process did not stop; preserve checkpoint')
    snapshot = OUT / 'relative_turn_contact_v1_resume'
    snapshot.mkdir()
    for name in ('checkpoint_state.npz', 'checkpoint_goals.npz', 'progress.json', 'start_state.npz'):
        (snapshot / name).write_bytes((OUT / 'relative_turn_contact_v1' / name).read_bytes())
    text = OLD.read_text(encoding='utf-8')
    changes = [
        ("target=upright_yaw_target(start_rotation,initial['up'],args.increment)",
         "target=upright_yaw_target(restore(model,dict(np.load(ROOT/'right_hand_rotation_20261112/stage3_preparation/relative_turn_contact_v1_resume/start_state.npz',allow_pickle=False))).geom_xmat[cube].reshape(3,3),initial['up'],90.)"),
        ('mujoco.mj_step(model,branch);stop=check.check(branch)',
         'mujoco.mj_step(model,branch);stop=check.check(branch) if (_+1)%5==0 else []'),
        ("status='stage3_relative_turn_model_teacher'", "status='stage3_relative_turn_teacher_fast_planning_actual1ms'"),
    ]
    for before, after in changes:
        if text.count(before) != 1: raise RuntimeError('Unexpected exact source edit')
        text = text.replace(before, after, 1)
    compile(text, str(NEW), 'exec'); NEW.write_text(text, encoding='utf-8')
    r = json.loads((snapshot / 'progress.json').read_bytes())
    report = {'source_sha256': hashlib.sha256(OLD.read_bytes()).hexdigest(), 'new_sha256': hashlib.sha256(NEW.read_bytes()).hexdigest(),
              'changes': changes, 'continued_after_goals': r['step'], 'remaining_error_deg': r['actual']['error_deg'],
              'only_hypothetical_prediction_guard_stride_ms': 5, 'actual_execution_guard_stride_ms': 1,
              'native_thresholds_unchanged': True, 'hardware_allowed': False}
    (snapshot / 'continuation.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({'continued_after_goals': r['step'], 'remaining_error_deg': r['actual']['error_deg']}))


if __name__ == '__main__': main()
