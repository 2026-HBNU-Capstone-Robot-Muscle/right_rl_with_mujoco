"""OS-backed instance lock; a second launch never opens camera or serial."""
import os
from pathlib import Path

class SingleInstance:
    def __init__(self,path):self.path=Path(path);self.file=None
    def __enter__(self):
        self.file=self.path.open('a+b')
        self.file.seek(0,2)
        if self.file.tell()==0:self.file.write(b'0');self.file.flush()
        self.file.seek(0)
        try:
            if os.name=='nt':
                import msvcrt
                msvcrt.locking(self.file.fileno(),msvcrt.LK_NBLCK,1)
            else:
                import fcntl
                fcntl.flock(self.file.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        except (OSError,IOError):
            self.file.close();self.file=None
            raise RuntimeError('Final grip program is already running. Use its existing terminal; no devices opened.')
        return self
    def __exit__(self,*args):
        if self.file is not None:
            self.file.seek(0)
            try:
                if os.name=='nt':
                    import msvcrt
                    msvcrt.locking(self.file.fileno(),msvcrt.LK_UNLCK,1)
                else:
                    import fcntl
                    fcntl.flock(self.file.fileno(),fcntl.LOCK_UN)
            finally:self.file.close();self.file=None
