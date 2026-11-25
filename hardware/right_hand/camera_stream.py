"""Named DirectShow capture in a disposable process. No motor imports or APIs.

OpenCV open/read/release can block in native code. A thread alone cannot enforce
a deadline; the parent can terminate this camera-only process without touching
the process that owns the motor bus. Frame timestamps originate after read(),
not when the parent receives them.
"""
import multiprocessing as mp
import queue
import threading
import time

EXPECTED_NAME = 'Streaming Webcam'  # Device used for camera1_720.npz calibration.


def select_camera(names):
    matches = [i for i, name in enumerate(names)
               if name.strip().casefold() == EXPECTED_NAME.casefold()]
    if len(matches) != 1:
        raise RuntimeError(f'Expected one {EXPECTED_NAME!r}, found {len(matches)}. '
                           f'Devices: {names}. No camera selected.')
    return matches[0]


def enumerate_names():
    from pygrabber.dshow_graph import FilterGraph
    graph = FilterGraph()
    try:
        return graph.get_input_devices()
    finally:
        graph.remove_filters()


def inventory_worker(send, stop):
    """Only enumerate devices; never construct a VideoCapture or motor bus."""
    try:
        names = enumerate_names()
        send.send(('devices', names))
        index = select_camera(names)
        meta = dict(camera_index=index, device_name=names[index], stream_started=False)
        send.send(('selected', meta))
        send.send(('ready', meta))
        stop.wait()
    except BaseException as exc:
        send.send(('error', f'{type(exc).__name__}: {exc}'))
    finally:
        send.close()


def read_frame(cap,stop):
    """Tolerate up to two transient read failures without reusing any old frame."""
    for attempt in range(3):
        if stop.is_set(): return None
        ok,frame=cap.read()
        stamp=time.monotonic()
        if ok and frame is not None:
            if frame.shape!=(720,1280,3): raise RuntimeError('Camera frame shape changed')
            return stamp,frame
        if attempt<2 and stop.wait(.03): return None
    raise RuntimeError('Camera read failed 3 consecutive times; check webcam connection or other camera apps')


def render_preview(cv,frame,preview,instruction):
    """GUI work belongs exclusively to the disposable camera process."""
    updates=preview['updates']
    while True:
        try:instruction=updates.get_nowait()
        except queue.Empty:break
    display=frame.copy()
    cv.putText(display,instruction,(10,30),cv.FONT_HERSHEY_SIMPLEX,.7,(0,255,255),2)
    cv.imshow(preview['title'],display)
    key=cv.waitKey(1)&0xff
    closed=cv.getWindowProperty(preview['title'],cv.WND_PROP_VISIBLE)<1
    return instruction,key in (ord('q'),ord('Q'),27) or closed


def camera_worker(send, stop, preview=None):
    cap = None
    cv = None
    try:
        names = enumerate_names()
        send.send(('devices', names))
        index = select_camera(names)
        send.send(('selected', dict(camera_index=index, device_name=names[index])))
        import cv2 as cv
        from camera_720_check import open_hd
        # Do not fall back to MSMF: indices can differ and a failed open may
        # take minutes. All native camera calls remain in this bounded worker.
        cap, meta, attempts = open_hd(cv, index, require_mjpg=True,
                                     backends=((cv.CAP_DSHOW, 'DSHOW'),))
        if preview is not None:
            cv.namedWindow(preview['title'],cv.WINDOW_NORMAL)
            cv.resizeWindow(preview['title'],1000,600)
        meta = dict(meta, camera_index=index, device_name=names[index],
                    attempts=[dict(row) for row in attempts])
        send.send(('ready', meta))
        instruction='EMPTY HAND';preview_open=preview is not None
        while not stop.is_set():
            packet=read_frame(cap,stop)
            if packet is None: break
            send.send(('frame', packet))
            if preview_open:
                instruction,requested_stop=render_preview(cv,packet[1],preview,instruction)
                if requested_stop:
                    send.send(('preview_stop','Camera user stop'))
                    preview_open=False
    except BaseException as exc:
        try:
            send.send(('error', f'{type(exc).__name__}: {exc}'))
        except (BrokenPipeError, EOFError, OSError):
            pass
    finally:
        # These native cleanup calls may also hang; the parent enforces exit.
        try:
            if cap is not None: cap.release()
        finally:
            if preview is not None and cv is not None:
                try:cv.destroyAllWindows()
                except Exception:pass
            send.close()


class CameraStream:
    def __init__(self, report, *, timeout=10., worker=camera_worker,preview_title=None):
        self.report = report
        self.lock = threading.Lock()
        self.ready = threading.Event()
        self.closing = False
        self.error = self.meta = self.latest = None
        self.sequence = self.used = 0
        self.preview_stop_requested=False;self.preview_updates=None;self.preview_instruction=None
        ctx = mp.get_context('spawn')
        self.stop = ctx.Event()
        self.receive, send = ctx.Pipe(duplex=False)
        worker_args=(send,self.stop)
        if preview_title is not None:
            self.preview_updates=ctx.Queue(maxsize=1)
            worker_args+=(dict(title=preview_title,updates=self.preview_updates),)
            report['camera_preview_owner']='camera_worker_process'
        self.process = ctx.Process(target=worker, args=worker_args, daemon=True)
        self.thread = None
        report['camera_startup'] = dict(expected_device=EXPECTED_NAME, backend='DSHOW',
                                        timeout_seconds=timeout, motors_accessed=False)
        print(f'Camera: locating {EXPECTED_NAME}; DSHOW 720p MJPG; startup limit {timeout:g}s.', flush=True)
        began = time.monotonic()
        try:
            self.process.start()
            send.close()
            self.thread = threading.Thread(target=self._receive, daemon=True)
            self.thread.start()
            while not self.ready.wait(.05):
                if time.monotonic()-began >= timeout:
                    raise RuntimeError(f'Camera startup exceeded {timeout:g}s; camera worker stopped. '
                                       'Close other camera apps and check the external webcam connection.')
            if self.error is not None: raise RuntimeError(self.error)
            if time.monotonic()-began >= timeout:
                raise RuntimeError(f'Camera startup exceeded {timeout:g}s')
        except BaseException as exc:
            report['camera_startup']['error'] = str(exc)
            self.release()
            send.close()
            raise

    def _receive(self):
        try:
            while not self.closing:
                kind, payload = self.receive.recv()
                if kind == 'devices':
                    self.report['camera_startup']['devices'] = payload
                    print(f'DSHOW devices: {list(enumerate(payload))}', flush=True)
                elif kind == 'selected':
                    self.report['camera_startup'].update(payload)
                    print(f'Selected camera {payload["camera_index"]}: {payload["device_name"]}', flush=True)
                elif kind == 'ready':
                    self.meta = payload
                    self.ready.set()
                elif kind == 'frame':
                    with self.lock:
                        self.sequence += 1
                        self.latest = (self.sequence, *payload)
                elif kind == 'error':
                    raise RuntimeError(payload)
                elif kind == 'preview_stop':
                    self.preview_stop_requested=True
                else:
                    raise RuntimeError(f'Unexpected camera worker message: {kind}')
        except (EOFError, OSError, RuntimeError) as exc:
            if not self.closing:
                with self.lock:
                    self.error = str(exc) or 'Camera worker exited unexpectedly'
                self.ready.set()

    def read_packet(self):
        with self.lock:
            if self.error is not None: raise RuntimeError(self.error)
            if self.latest is None or self.latest[0] == self.used: return None
            self.used, stamp, frame = self.latest
            return stamp, frame

    def check_preview(self,instruction):
        """Nonblocking metadata/Q polling; never calls OpenCV or waits for GUI."""
        if self.preview_stop_requested:raise KeyboardInterrupt('Camera user stop')
        if self.error is not None:raise RuntimeError(self.error)
        if self.preview_updates is not None and instruction!=self.preview_instruction:
            try:self.preview_updates.put_nowait(instruction)
            except queue.Full:pass
            else:self.preview_instruction=instruction

    def release(self):
        self.closing = True
        self.stop.set()
        if self.process.pid is not None:
            self.process.join(timeout=1.)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(timeout=.5)
            if self.process.is_alive():
                self.process.kill()
                self.process.join(timeout=.5)
            self.report['camera_worker_stopped'] = not self.process.is_alive()
        if self.thread is not None: self.thread.join(timeout=.2)
        self.receive.close()
        if self.preview_updates is not None:
            self.preview_updates.cancel_join_thread()
            self.preview_updates.close()


if __name__ == '__main__':
    import argparse
    import json
    parser = argparse.ArgumentParser(description='Bounded camera name lookup only; no stream or motor access.')
    parser.add_argument('--list', action='store_true', required=True)
    parser.parse_args()
    report = {}
    try:
        stream = CameraStream(report, worker=inventory_worker)
        stream.release()
    except Exception as exc:
        print(f'Camera lookup failed: {exc}', flush=True)
        raise SystemExit(1)
    print(json.dumps(report, ensure_ascii=False, indent=2))
