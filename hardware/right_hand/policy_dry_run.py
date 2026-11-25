"""Run the deployment loop with camera 1 but no Dynamixel connection or writes."""

import os
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parent / "work" / "Leap_Test"
LOG = ROOT / "policy_dry_run_result.txt"


def main():
    env = os.environ.copy()
    extras = [PROJECT / ".cv4", PROJECT / ".deps", PROJECT / ".hw_runtime_10"]
    env["PYTHONPATH"] = os.pathsep.join(str(p) for p in extras)
    env["PYTHONIOENCODING"] = "utf-8"
    env["PYTHONUNBUFFERED"] = "1"
    command = [
        sys.executable, str(PROJECT / "deploy" / "policy_runner.py"),
        "--dry_run", "--cams", f"1:{ROOT / 'camera1.npz'}",
        "--face_rot", "1,3,3,2,1,3", "--seconds", "10",
        "--max_stale", "0",
    ]
    print("READ-ONLY DRY RUN: no motor port is opened and no servo command is sent.", flush=True)
    print("Keep the stationary cube and hand markers in the webcam view.", flush=True)
    with LOG.open("w", encoding="utf-8") as log:
        with subprocess.Popen(command, cwd=PROJECT, env=env,
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              text=True, encoding="utf-8", errors="replace") as process:
            for line in process.stdout:
                print(line, end="", flush=True)
                log.write(line)
            code = process.wait()
    print(f"Result saved: {LOG}", flush=True)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
