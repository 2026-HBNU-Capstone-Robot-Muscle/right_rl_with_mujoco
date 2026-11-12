from pathlib import Path
import json
root=Path(__file__).resolve().parent
folder=root/'horizontal_quarter_trial_20261027_152632_508556/camera'
for path in folder.iterdir():
    if path.is_file():
        print(path.name,path.stat().st_size)
        if path.suffix in ('.json','.jsonl'):
            with path.open(encoding='utf-8-sig') as f:
                for _,line in zip(range(2),f):print(line[:2400])
    else:print(path.name,'directory',len(list(path.iterdir())))
