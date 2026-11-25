"""Bounded, cancellable OFF-only checker process for this longer empty route."""
import subprocess,time
from pathlib import Path
from thumb_live_start_gate import wsl_path
MAX_WALL_SECONDS=240
def run_screen(request,output,*,cancel,sleep=time.sleep,now=time.monotonic,popen=subprocess.Popen,checker,failure_details):
    request,output=Path(request),Path(output)
    if output.exists():raise RuntimeError('Screen output must be new; no stale proof reuse')
    command=['wsl.exe','--exec','/home/star1209042/venv/bin/python','-B',wsl_path(checker),wsl_path(request),wsl_path(output)]
    log=output.with_suffix('.log');began=now();last_progress=began
    with log.open('wb') as stream:
        process=popen(command,stdout=stream,stderr=subprocess.STDOUT,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        try:
            while process.poll() is None:
                cancel()
                if now()-began>MAX_WALL_SECONDS:raise RuntimeError(f'OFF path screen exceeded {MAX_WALL_SECONDS}s; no writes')
                if now()-last_progress>5:
                    print(f'OFF path check running ({now()-began:.0f}s / {MAX_WALL_SECONDS}s limit); Q cancels.',flush=True)
                    last_progress=now()
                sleep(.05)
            if process.returncode!=0:
                raise RuntimeError('Current-pose path screen failed: '+failure_details(output)+'; no motion started; see '+str(log))
        finally:
            if process.poll() is None:
                process.terminate()
                try:process.wait(timeout=3)
                except subprocess.TimeoutExpired:process.kill();process.wait(timeout=3)
    if not output.is_file():raise RuntimeError('No current-pose proof generated; no writes')
    return output
