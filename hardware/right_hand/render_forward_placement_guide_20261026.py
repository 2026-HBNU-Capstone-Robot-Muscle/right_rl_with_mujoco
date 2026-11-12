"""Render the local HTML/SVG engineering guide in an isolated headless browser."""
import json
import subprocess
import tempfile
from pathlib import Path
from forward_cube_placement_20261026 import REVIEW

def main():
    browsers = [Path('C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe'),
                Path('C:/Program Files/Google/Chrome/Application/chrome.exe')]
    browser = next((p for p in browsers if p.is_file()), None)
    if browser is None:
        print('No local headless browser found; standalone HTML guide remains available.')
        return
    folder = REVIEW.parent
    page = folder/'forward_placement_guide.html'
    output = folder/'forward_placement_guide.png'
    profile = Path(tempfile.mkdtemp(prefix='leap_placement_diagram_')).resolve()
    if profile.parent != Path(tempfile.gettempdir()).resolve():
        raise ValueError('Unexpected browser profile directory')
    # This helper never touches the user's browser profile. The temporary
    # profile is left for OS temporary-file cleanup; no recursive delete runs.
    cmd = [str(browser), '--headless', '--disable-gpu', '--hide-scrollbars',
        '--disable-background-networking', '--disable-component-update', '--disable-extensions',
        '--no-first-run', '--no-default-browser-check', '--user-data-dir='+str(profile),
        '--window-size=1280,1380', '--screenshot='+str(output), page.as_uri()]
    r = subprocess.run(cmd, capture_output=True, text=True, errors='replace', timeout=40,
        creationflags=subprocess.CREATE_NO_WINDOW)
    print(json.dumps(dict(browser=browser.name, exit_code=r.returncode,
        screenshot=str(output) if output.is_file() else None,
        local_html_svg_render_only=True, no_motor_or_camera_access=True), ensure_ascii=False))

if __name__ == '__main__':main()
