"""Next EMPTY thumb segment from the completed, recorded prefix.

The reference is the saved read-only OFF preflight, not the previous ON pose.
Fresh OFF readings must match the reviewed neighborhood before any write.
No raw angle wrap, zero change, or native simulation range edit is performed.
"""
import hashlib
import json
from pathlib import Path
import numpy as np
from hand_motor_mapping import SIM_TO_MOTOR_IDS

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT/'thumb_continuation_empty_20261021_192403_090594/result.json'
SOURCE_SHA = 'bd5847e479530711379b5e9a4519cfff6e5b96aaff4a4bbd39e4dca274d1cb16'
ROOTS = np.array([7, 11, 15])
THUMB = np.arange(4)
FROZEN = np.array([i for i in range(16) if i not in (*ROOTS, *THUMB)])
TOLERANCE = 32
MAX_THUMB_TRAVEL = 384
MAX_GOAL_STEP = 2
ROOT_TARGET = 2472
THUMB_TARGET = np.array([2601, 3100, 3074, 2708])
ROOT_STEPS, THUMB_STEPS = 120, 240
PLAN_VERSION = 'thumb_continuation_empty_v2_off_reference'
REVIEW_PATH = ROOT/'right_hand_preparation_20261021/thumb_continuation_screen.json'
TITLE = 'EMPTY THUMB CONTINUATION v2: reviewed OFF pose -> roots -> thumb -> hold -> OFF.'
DESCRIPTION = 'Thumb IDs1..4 max 384 ticks (33.75deg); slow interpolation. Cube stays OUT.'
FOLDER_PREFIX = 'thumb_continuation_empty'
STATUS = 'empty_thumb_continuation_complete'
COMPLETE_MESSAGE = 'Thumb continuation complete. Torque OFF verified; settings restored.'


def source_position():
    content = SOURCE.read_bytes()
    if hashlib.sha256(content).hexdigest() != SOURCE_SHA:
        raise ValueError('Reviewed OFF preflight source changed')
    r = json.loads(content)
    if (r['status'] != 'stopped' or r['failed_phase'] != 'preflight'
            or r['write_events'] != [] or not r['all_torque_off_verified']
            or r['camera_recording']['status'] != 'complete'
            or r['joint_mapping']['motor_ids_in_sim_order'] != list(SIM_TO_MOTOR_IDS)):
        raise ValueError('Reviewed source must be a recorded read-only OFF preflight')
    s = r['preflight_state']
    if (any(any(s[k]) for k in ('torque', 'error', 'watchdog'))
            or min(s['voltage']) < 38 or max(s['voltage']) > 65
            or max(s['temperature']) > 45):
        raise ValueError('Reviewed OFF source is not healthy')
    a = np.asarray(s['position'])
    if a.shape != (16,) or not np.isfinite(a).all() or np.any(a != np.rint(a)):
        raise ValueError('Invalid reviewed OFF position')
    return a.astype(np.int64)


def validate_start(raw):
    a = np.asarray(raw)
    if a.shape != (16,) or not np.isfinite(a).all() or np.any(a != np.rint(a)):
        raise ValueError('Invalid raw encoder array')
    reference = source_position()
    bad = np.flatnonzero(abs(a-reference) > TOLERANCE)
    if len(bad):
        raise ValueError('Current OFF pose differs from the recorded continuation start; no writes: '
                         + '; '.join(f'ID{i+1} {int(a[i]-reference[i]):+d} ticks' for i in bad))
    return a.astype(np.int64)


def plan(raw):
    a = validate_start(raw)
    root = a.copy()
    # If a root relaxed slightly above 2472, hold it; never bend it backwards.
    root[ROOTS] = np.maximum(a[ROOTS], ROOT_TARGET)
    end = root.copy()
    delta = THUMB_TARGET-a[:4]
    end[:4] = a[:4] + np.rint(delta*min(1., MAX_THUMB_TRAVEL/max(1, max(abs(delta))))).astype(int)
    rows = []
    for phase, b, count in [('stage_0_roots', root, ROOT_STEPS),
                            ('stage_1_thumb_prefix', end, THUMB_STEPS)]:
        for k in range(1, count+1):
            rows.append((phase, np.rint(a+(b-a)*k/count).astype(np.int64)))
        a = b.copy()
    return rows


def plan_digest(raw):
    payload = [(phase, q.tolist()) for phase, q in plan(raw)]
    return hashlib.sha256(json.dumps(payload, separators=(',', ':')).encode()).hexdigest()
