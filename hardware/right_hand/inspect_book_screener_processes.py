"""List only this workspace's native book screener processes. Read-only."""
import json
from pathlib import Path

root=Path(__file__).resolve().parent
target=str(root/'screen_book_empty_preparation.py');rows=[]
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():continue
    try:
        args=(proc/'cmdline').read_bytes().decode(errors='replace').split('\0')
        if target not in args:continue
        index=args.index(target);request=args[index+1] if len(args)>index+1 else None
        rows.append(dict(pid=int(proc.name),script='screen_book_empty_preparation.py',request=request))
    except (OSError,ValueError):continue
print(json.dumps(dict(owned_screener_processes=rows,process_changes_sent=0),indent=2))
