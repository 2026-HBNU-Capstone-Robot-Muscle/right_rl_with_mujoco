"""Publish already-read motor samples; this worker never accesses a device."""
from pathlib import Path
import json, threading, time


class LiveState:
    def __init__(self, folder, report):
        self.folder, self.report = Path(folder), report
        self.stop = threading.Event()
        self.error = None
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def snapshot(self, active=True):
        samples = self.report.get('samples', [])
        if not samples:
            return None
        row = dict(samples[-1])
        return dict(active=active, phase=row['phase'],
            sample_monotonic=self.report['enabled_at']+row['t'],
            published_monotonic=time.monotonic(),
            goal_ticks=row['goal'], actual_ticks=row['actual'],
            torque=row['torque'], hardware_error=row['error'], current_ma=row['current_ma'],
            health_read_ms=row['read_ms'], source='existing validated motor sample; no extra motor reads')

    def _write(self, value):
        pending = self.folder/'live_state.pending'
        pending.write_text(json.dumps(value, ensure_ascii=False)+'\n', encoding='utf-8')
        pending.replace(self.folder/'live_state.json')

    def _run(self):
        while not self.stop.wait(.25):
            try:
                value = self.snapshot()
                if value is not None:
                    self._write(value)
            except Exception as exc:
                self.error = str(exc)

    def close(self):
        self.stop.set()
        self.thread.join(.5)
        if not self.thread.is_alive():
            value = self.snapshot(active=False)
            if value is not None:
                try:
                    self._write(value)
                except Exception as exc:
                    self.error = str(exc)
        return dict(worker_stopped=not self.thread.is_alive(), error=self.error,
                    motor_reads_added=0, motor_writes_added=0)
