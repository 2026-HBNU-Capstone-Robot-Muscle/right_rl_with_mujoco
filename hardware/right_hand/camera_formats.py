"""Enumerate the external webcam's advertised formats; never start streaming.

Uses pygrabber FilterGraph / VideoInput.get_formats (installed .camdiag).
No format changes, calibration reads, camera images, or motor access.
"""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def normalize_format(item):
    if not isinstance(item, dict):
        return {'raw': item}
    result = dict(item)
    # Some pygrabber versions label interval-derived fps endpoints in reverse.
    rates = [float(item[k]) for k in ('min_framerate', 'max_framerate')
             if item.get(k) is not None]
    if rates:
        result['fps_range_sorted'] = [min(rates), max(rates)]
    return result


def main():
    graph = None
    report = {'stream_started': False, 'settings_changed': False, 'motors_accessed': False}
    code = 0
    print('READ ONLY: device names and supported formats. No preview or motor access.', flush=True)
    try:
        from pygrabber.dshow_graph import FilterGraph
        graph = FilterGraph()
        names = graph.get_input_devices()
        report['devices'] = names
        print('Devices:', names, flush=True)
        matches = [i for i, name in enumerate(names) if name.strip().lower() == 'streaming webcam']
        if len(matches) != 1:
            raise RuntimeError('Expected exactly one Streaming Webcam; no device selected.')
        index = matches[0]
        report.update(camera_index=index, device_name=names[index])
        graph.add_video_input_device(index)
        formats = graph.get_input_device().get_formats()
        report['formats'] = [normalize_format(item) for item in formats]
        report['status'] = 'complete'
        print('Selected:', names[index], flush=True)
        for item in report['formats']:
            print(json.dumps(item, ensure_ascii=False), flush=True)
    except Exception as exc:
        report.update(status='error', error=f'{type(exc).__name__}: {exc}')
        print(report['error'], flush=True)
        code = 1
    finally:
        if graph is not None:
            try:
                graph.remove_filters()
            except Exception as exc:
                report['cleanup_warning'] = str(exc)
        output = ROOT / 'camera_formats_result.json'
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        print(f'Saved: {output}', flush=True)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
