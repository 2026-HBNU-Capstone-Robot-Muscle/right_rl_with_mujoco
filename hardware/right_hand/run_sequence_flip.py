"""Run the user's original sequence with an embedded-Python import path fix.

The bundled Python uses python312._pth and does not add a script's directory
to sys.path. This wrapper changes imports only; the source sequence is untouched.
"""
import runpy
import sys
from pathlib import Path


def main():
    project_python = Path.home() / 'Documents' / 'GitHub' / 'LEAP_Hand_API' / 'python'
    script = project_python / 'sequence_flip.py'
    if not script.is_file() or not (project_python / 'main.py').is_file():
        raise FileNotFoundError(f'Missing sequence_flip.py or main.py in {project_python}')
    sys.path.insert(0, str(project_python))
    import main as leap_main
    if not hasattr(leap_main, 'LeapNode'):
        raise ImportError(f'LeapNode missing from {project_python / "main.py"}')
    if sys.argv[1:] == ['--check']:
        print(f'Import OK: {project_python / "main.py"}; motor port NOT opened.', flush=True)
        return 0
    sys.argv = [str(script)]
    runpy.run_path(str(script), run_name='__main__')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
