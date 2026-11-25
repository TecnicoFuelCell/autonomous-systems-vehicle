"""Build camera calibration messages from the JSON files written by vehicle_calibration."""

import json
from pathlib import Path

from car_msgs.msg import CameraExtrinsics
from sensor_msgs.msg import CameraInfo


def _load_json(path):
    with Path(path).open(encoding='utf-8') as handle:
        return json.load(handle)


def load_camera_info(path, frame_id):
    """CameraInfo from the intrinsics file (header stamp left empty).

    width/height are 0 when the file does not record the calibrated resolution.
    """
    document = _load_json(path)
    mtx = document['camera_matrix']

    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.width = int(document.get('width', 0))
    msg.height = int(document.get('height', 0))
    msg.k = [
        float(mtx[0][0]), 0.0, float(mtx[0][2]),
        0.0, float(mtx[1][1]), float(mtx[1][2]),
        0.0, 0.0, 1.0,
    ]
    msg.p = [
        float(mtx[0][0]), 0.0, float(mtx[0][2]), 0.0,
        0.0, float(mtx[1][1]), float(mtx[1][2]), 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]
    # Monocular camera: no rectification, so R is the identity (all zeros is invalid).
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.distortion_model = 'plumb_bob'
    msg.d = [float(value) for value in document['distortion_coefficients']]
    return msg


def load_camera_extrinsics(path, parent_frame):
    """CameraExtrinsics from the extrinsics file (header stamp left empty)."""
    document = _load_json(path)

    msg = CameraExtrinsics()
    msg.header.frame_id = parent_frame
    msg.rotation_matrix = [float(value) for value in document['rotation_matrix']]
    msg.translation = [float(value) for value in document['translation']]
    return msg
