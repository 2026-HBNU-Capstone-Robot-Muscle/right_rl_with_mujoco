"""Read latest user-pasted console log without altering it."""

from pathlib import Path

folder = Path.home() / '.codex' / 'attachments' / 'af1de295-f480-4c2e-86b3-b516aab27ad6'
for path in folder.glob('*.txt'):
    lines = path.read_text(encoding='utf-8', errors='replace').splitlines()
    print(path.name, f'{len(lines)} lines')
    for line in lines[-60:]:
        print(line)
