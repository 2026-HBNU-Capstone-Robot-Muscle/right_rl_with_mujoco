# Cloud project continuation

Read `CURRENT_STATE.md` and `cloud_handoff_20261106/CLOUD_START_PROMPT.md` first. Respond to the user in Korean.

This repository is a selected snapshot of a larger Windows robotics workspace. `cloud/upload_manifest.json` lists the exact copied source bytes and omitted categories. Historic scripts can reference files that were not selected. Inspect dependencies before claiming an old experiment can run.

The cloud task is for offline analysis, code changes and MuJoCo simulation. Do not execute hardware `.cmd` files, `--execute` entry points, serial/Dynamixel writes, torque commands or camera capture. The original robot USB and webcam are not connected to the cloud. Preserve the user's original-PC hardware workflow separately.

Start with `python -B cloud/smoke.py --verify-only`; after installing `requirements-cloud.txt`, run `python -B cloud/smoke.py`. These commands do not open robot or camera devices. This smoke check verifies the snapshot and saved model, not task success or full controller admission.

Keep frozen evidence unchanged. If modifying hash-bound source or paths, produce new offline reviews; do not edit old reports to imply they still approve changed code. Do not weaken native joint bounds, current, voltage or tracking guards. Keep failed simulation conditions visible and distinguish physical observations, nominal model results, search proposals and fully evaluated paths.

The latest middle-only physical probe completed with torque OFF and settings restored, but its photo/video effect is still pending review. The overall 90-degree direction-change and lower-body grip integration task is not complete. Follow the latest state rather than older 'next action' entries in historical progress notes.

Preserve copied source line endings: `.gitattributes` disables text normalization so frozen SHA256 bindings remain valid. Never commit `.runtime`, account configuration, tokens or raw session histories. Request missing source assets when required instead of inventing results.
