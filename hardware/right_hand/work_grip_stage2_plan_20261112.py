"""Immutable stage-2 diagnostic: observed first turn -> work-grip probe -> hold."""
import hashlib
import json
from pathlib import Path
import numpy as np
from forward_hold_plan_20261104 import ForwardHoldPlan, validate_candidate as first_candidate, hashes as first_hashes
from single_forward_plan_20261030 import RawBase
from calibrated_empty_ring_margin_20261028 import rows as prefix_rows
from loaded_quarter_reentry_20261025 import limits, validate_path
from new_pair_grasp_plan_20261027 import ENTRY

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "right_hand_rotation_20261112/stage2_work_grip"
CANDIDATE = OUT / "candidate.npz"
MANIFEST = OUT / "candidate_manifest.json"
REQUEST = OUT / "request.json"
PROOF = OUT / "review.json"
MOVE = "stage_11_work_grip_transition"
HOLD = "stage_12_work_grip_hold"
HOLD_SAMPLES = 134
FILES = ("work_grip_stage2_plan_20261112.py", "work_grip_stage2_gate_20261112.py",
         "work_grip_stage2_capture_20261112.py", "leap_cycle_stage2_work_grip_20261112.py",
         "leap_cycle_stage2_work_grip.cmd", "screen_work_grip_stage2_current_20261112.py",
         "screen_work_grip_stage2_loaded_20261112.py", "screen_work_grip_stage2_hand_20261112.py")


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_candidate():
    manifest = json.loads(MANIFEST.read_bytes())
    if sha(CANDIDATE) != manifest["candidate_sha256"]:
        raise RuntimeError("Stage-2 candidate changed")
    for name, digest in manifest["source_files_sha256"].items():
        if sha(ROOT / name) != digest:
            raise RuntimeError("Stage-2 source changed: " + name)
    z = dict(np.load(CANDIDATE, allow_pickle=False))
    validate_arrays(z, first_candidate(), manifest)
    return z


def validate_arrays(z, old, manifest):
    q, phases = z["goals"], z["phases"]
    lo, hi = limits()
    if (q.ndim != 2 or q.shape[1] != 16 or phases.shape != (len(q),)
            or not np.isfinite(q).all() or np.any(q != np.rint(q))
            or np.any(q < lo) or np.any(q > hi) or not np.array_equal(z["start"], old["start"])
            or np.max(abs(np.diff(np.vstack([z["start"], q]), axis=0))) > 2):
        raise RuntimeError("Stage-2 native limits/rate/start rejected")
    n = len(old["goals"])
    if not np.array_equal(q[:n], old["goals"]) or not np.array_equal(phases[:n], old["phases"]):
        raise RuntimeError("Actually executed first-turn goals changed")
    if list(dict.fromkeys(phases[n:].tolist())) != [MOVE, HOLD]:
        raise RuntimeError("Stage-2 diagnostic scope changed")
    held = q[phases == HOLD]
    if len(held) != HOLD_SAMPLES or not np.all(held == held[0]):
        raise RuntimeError("Stage-2 fixed hold changed")
    if len(q) != manifest["loaded_goals"] or manifest["second_rotation_included"]:
        raise RuntimeError("Stage-2 length/scope mismatch")
    return z


def hashes():
    paths = [*(ROOT / name for name in FILES), CANDIDATE, MANIFEST, REQUEST]
    paths += sorted((OUT / "loaded_models").glob("gain*/review.json"))
    paths += sorted((OUT / "hand_models").glob("gain*.json"))
    return {**first_hashes(), **{p.relative_to(ROOT).as_posix(): sha(p) for p in paths}}


class WorkGripStage2Plan(ForwardHoldPlan):
    PLAN_VERSION = "observed_first_turn_then_work_grip_diagnostic_v1_20261112"
    TITLE = "첫 회전 -> 10초 유지 -> 파지 전환 관찰 -> 10초 유지. 다음 회전은 포함하지 않습니다."
    FOLDER_PREFIX = "work_grip_stage2"
    STATUS = "work_grip_stage2_observation_complete"
    COMPLETE_MESSAGE = "파지 전환 관찰 종료. 토크OFF와 설정 복구를 기록했습니다. 실제 공간·방향·지지는 저장 영상으로 검토합니다."
    proof_hashes = staticmethod(hashes)

    def __init__(self, request):
        RawBase.__init__(self, request)
        self.raw_off_ticks = self.center.copy()
        self.review_center_reused = False

    def plan(self, raw):
        a = self.validate_start(raw)
        z = validate_candidate()
        rows = prefix_rows(self, a) + [(str(p), q.copy()) for p, q in zip(z["phases"], z["goals"])]
        values = validate_path(a, rows, ENTRY)
        if np.max(abs(values[:, :4] - a[:4])) > self.MAX_THUMB_TRAVEL:
            raise RuntimeError("Existing thumb travel bound exceeded")
        if (len(rows) + 40) * self.SAMPLE_PERIOD + 20 > self.ON_TIMEOUT:
            raise RuntimeError("Existing 300-second motion budget exceeded")
        return rows

    def frozen_for_phase(self, phase):
        if phase == MOVE:
            return np.array([], dtype=int)
        if phase == HOLD:
            return np.arange(16)
        return super().frozen_for_phase(phase)

    def progress_for_phase(self, phase):
        if phase == MOVE:
            return np.arange(16)
        if phase == HOLD:
            return np.array([], dtype=int)
        return super().progress_for_phase(phase)
