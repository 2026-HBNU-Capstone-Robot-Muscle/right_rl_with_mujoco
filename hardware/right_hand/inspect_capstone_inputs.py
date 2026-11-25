import pathlib, sys, importlib.util
sys.stdout.reconfigure(encoding='utf-8')
home = pathlib.Path.home()
print('HOME', home)
for p in [home / 'Documents' / '카카오톡 받은 파일' / '(양식)캡스톤디자인 과제 결과보고서.hwp', home / 'Downloads' / 'ROQET_04_발표영상.mp4']:
    print('FILE', p, p.exists(), p.stat().st_size if p.exists() else '')
for name in ['olefile', 'cv2', 'PIL', 'imageio_ffmpeg', 'numpy', 'whisper', 'faster_whisper']:
    print('MODULE', name, bool(importlib.util.find_spec(name)))
for parent in [pathlib.Path.cwd(), *pathlib.Path.cwd().parents]:
    p = parent / 'AGENTS.md'
    if p.exists():
        print('AGENTS', p, p.read_text(encoding='utf-8'))
