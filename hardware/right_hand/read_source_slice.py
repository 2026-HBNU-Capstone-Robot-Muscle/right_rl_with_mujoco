"""Small local source reader for environments without rg."""
from pathlib import Path
import sys
path=Path(sys.argv[1]);rows=path.read_text(encoding='utf-8-sig').splitlines()
if len(sys.argv)>2 and sys.argv[2].isdigit():
    first=int(sys.argv[2]);last=int(sys.argv[3]) if len(sys.argv)>3 else first+60
    selected=range(first-1,min(last,len(rows)))
else:
    terms=sys.argv[2:]
    selected=[i for i,row in enumerate(rows) if any(term in row for term in terms)]
for i in selected:print(f'{i+1}: {rows[i]}')
