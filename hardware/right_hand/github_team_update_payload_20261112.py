"""Emit explicitly selected public code/docs; no account files or raw records."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
paths={
    'hardware/right_hand/tip_first_transfer_20261112.py':ROOT/'tip_first_transfer_20261112.py',
    'hardware/right_hand/test_tip_first_transfer_20261112.py':ROOT/'test_tip_first_transfer_20261112.py',
    'hardware/right_hand/README.md':ROOT/'team_integration_20261112/README.md',
}
print(json.dumps([dict(path=target,mode='100644',type='blob',content=source.read_text(encoding='utf-8')) for target,source in paths.items()],ensure_ascii=False))
