"""
This auxiliary code is used to obtain the device ID of the camera. 
The device ID is stored next to intrisic matrices in the output. 
The device ID can be used in the future to identify if the matrices
belong to that device, therefore avoid unnecessary duplicate calibration.
"""

import re
import subprocess
from pathlib import Path


def probe_v4l2_device_id(device_path: str = '/dev/video0') -> str:
    device_path = str(device_path)
    try:
        result = subprocess.run(
            ['v4l2-ctl', '--device', device_path, '--info'],
            capture_output=True,
            text=True,
            check=True,
            timeout=5,
        )
        text = result.stdout + result.stderr
        card = _match_line(text, r'Card type\s*:\s*(.+)')
        bus = _match_line(text, r'Bus info\s*:\s*(.+)')
        if card and bus:
            return f'{card.strip()}|{bus.strip()}'
        if bus:
            return bus.strip()
    except (FileNotFoundError, subprocess.SubprocessError, OSError):
        pass
    return f'unknown:{device_path}'


def _match_line(text: str, pattern: str) -> str | None:
    match = re.search(pattern, text)
    return match.group(1).strip() if match else None


def device_path_from_video_index(index: int) -> str:
    return f'/dev/video{index}'
