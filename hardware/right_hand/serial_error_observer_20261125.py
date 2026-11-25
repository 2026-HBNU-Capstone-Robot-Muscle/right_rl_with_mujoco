"""Observe existing pyserial ClearCommError results without adding driver calls."""
import ctypes
import os
import time


def handle_value(value):
    return int(getattr(value, 'value', value) or 0)


class SerialErrorObserver:
    def __init__(self, serial_port, report, win32):
        self.win32 = win32
        self.handle = handle_value(serial_port._port_handle)
        self.original = win32.ClearCommError
        self.summary = report.setdefault('windows_uart_observer', dict(
            existing_ClearCommError_calls_observed=0, driver_error_events=[],
            new_driver_calls=0, driver_or_registry_settings_changed=False,
            baudrate=serial_port.baudrate, read_timeout=serial_port.timeout))

        def observe(handle, error_pointer, stat_pointer):
            # Preserve failure behavior and its original Windows error state.
            result = self.original(handle, error_pointer, stat_pointer)
            if not result:
                return result
            try:
                if handle_value(handle) == self.handle:
                    self.summary['existing_ClearCommError_calls_observed'] += 1
                    flags = int(ctypes.cast(error_pointer, ctypes.POINTER(win32.DWORD)).contents.value)
                    if flags:
                        events = self.summary['driver_error_events']
                        if len(events) < 64:
                            stat = ctypes.cast(stat_pointer, ctypes.POINTER(win32.COMSTAT)).contents
                            names = [name for name in ('CE_BREAK', 'CE_FRAME', 'CE_IOE', 'CE_OVERRUN',
                                'CE_RXOVER', 'CE_RXPARITY', 'CE_TXFULL')
                                if getattr(win32, name, 0) & flags]
                            events.append(dict(qpc_seconds=time.perf_counter(), error_mask=flags,
                                error_mask_hex=hex(flags), names_if_defined_by_pyserial=names,
                                input_queue_bytes=int(stat.cbInQue), output_queue_bytes=int(stat.cbOutQue)))
            except Exception:
                # Observation must not change the serial operation's outcome.
                self.summary['observer_decode_errors'] = self.summary.get('observer_decode_errors', 0)+1
            return result

        self.wrapper = observe
        win32.ClearCommError = observe

    def close(self):
        if self.win32.ClearCommError is self.wrapper:
            self.win32.ClearCommError = self.original


def attach(serial_port, report):
    if os.name != 'nt':
        report['windows_uart_observer'] = dict(supported=False, new_driver_calls=0)
        return None
    import serial.serialwin32 as native
    return SerialErrorObserver(serial_port, report, native.win32)
