"""Read/write on-vehicle calibration JSON files (flat layout, no versioning envelope)."""

import json
from pathlib import Path
from typing import Any

import numpy as np


def _write_json(path: str | Path, document: dict[str, Any]) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as handle:
        json.dump(document, handle, indent=2)
        handle.write('\n')
    return path


def _load_json(path: str | Path) -> dict[str, Any]:
    with Path(path).open('r', encoding='utf-8') as handle:
        return json.load(handle)


def write_camera_intrinsics(
    path: str | Path,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    width: int,
    height: int,
    device_id: str = '',
) -> Path:
    dist = np.asarray(distortion, dtype=np.float64).reshape(-1).tolist()
    document: dict[str, Any] = {
        'distortion_model': 'plumb_bob',
        'camera_matrix': np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3).tolist(),
        'distortion_coefficients': dist,
        'width': int(width),
        'height': int(height),
    }
    if device_id:
        document['device_id'] = device_id
    return _write_json(path, document)


def load_camera_intrinsics(path: str | Path) -> tuple[np.ndarray, np.ndarray, int, int]:
    document = _load_json(path)
    mtx = np.array(document['camera_matrix'], dtype=np.float64).reshape(3, 3)
    dist = np.array(document['distortion_coefficients'], dtype=np.float64).reshape(-1)
    width = int(document.get('width', 0))
    height = int(document.get('height', 0))
    return mtx, dist, width, height


def write_camera_extrinsics(
    path: str | Path,
    rotation_matrix: np.ndarray,
    translation: np.ndarray,
) -> Path:
    document = {
        'rotation_matrix': np.asarray(rotation_matrix, dtype=np.float64).reshape(-1).tolist(),
        'translation': np.asarray(translation, dtype=np.float64).reshape(-1).tolist(),
    }
    return _write_json(path, document)


def load_camera_extrinsics(path: str | Path) -> tuple[list[float], list[float]]:
    document = _load_json(path)
    return list(document['rotation_matrix']), list(document['translation'])


def write_magnetometer_calibration(path: str | Path, mag_data: dict[str, Any]) -> Path:
    return _write_json(path, mag_data)


def load_magnetometer_calibration(path: str | Path) -> dict[str, Any]:
    return _load_json(path)
