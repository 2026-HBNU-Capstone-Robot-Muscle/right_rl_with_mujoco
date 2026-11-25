"""Cancellable OFF-only check for the longer setup; active motion stays300s."""
import subprocess,time
from pathlib import Path
from thumb_live_start_gate import wsl_path
MAX_WALL_SECONDS=600
def run_screen(request,output,*,cancel,sleep=time.sleep,now=time.monotonic,popen=subprocess.Popen,checker,failure_details):
    request,output=Path(request),Path(output)
    if output.exists():raise RuntimeError('Screen output must be new')
    command=['wsl.exe','--exec','/home/star1209042/venv/bin/python','-B',wsl_path(checker),wsl_path(request),wsl_path(output)]
    log=output.with_suffix('.log');began=last=now()
    with log.open('wb') as stream:
        process=popen(command,stdout=stream,stderr=subprocess.STDOUT,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        try:
            while process.poll() is None:
                cancel()
                if now()-began>MAX_WALL_SECONDS:raise RuntimeError('OFF-only model check exceeded600s; no writes')
                if now()-last>5:
                    print(f'OFF path check running ({now()-began:.0f}s /600s); Q cancels. Motors remain OFF.',flush=True);last=now()
                sleep(.05)
            if process.returncode!=0:raise RuntimeError('Current setup screen failed: '+failure_details(output)+'; see '+str(log))
        finally:
            if process.poll() is None:
                process.terminate()
                try:process.wait(timeout=3)
                except subprocess.TimeoutExpired:process.kill();process.wait(timeout=3)
    if not output.is_file():raise RuntimeError('No current-pose proof generated')
    return output
