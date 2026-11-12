#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-cloud.txt
.venv/bin/python -B cloud/smoke.py
