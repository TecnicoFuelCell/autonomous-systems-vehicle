import argparse
import json
from pathlib import Path

import numpy as np

_DEFAULT_INTRINSICS_PATH = 'calibration/camera_intrinsics.json'

MATRIX = np.array(
    [
        [886.01450458, 0.0, 716.75440229],
        [0.0, 887.4856039, 366.32572717],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)
DEFAULT_DIST = np.array(
    [[0.14110704, -1.49049694, 0.00728032, 0.01576472, 4.66754431]],
    dtype=np.float64,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Write camera intrinsics JSON from hard-coded matrix values.',
    )
    # Required on purpose: a default would silently overwrite the real
    # calibration result at calibration/camera_intrinsics.json.
    parser.add_argument(
        '--output',
        type=str,
        required=True,
        help=f'Output JSON path (the live calibration file is {_DEFAULT_INTRINSICS_PATH})',
    )
    parser.add_argument('--device-id', default='manual_entry')
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    return parser


def main():
    args = parse_args().parse_args()
    out_path = Path(args.output)
    document = {
        'distortion_model': 'plumb_bob',
        'camera_matrix': MATRIX.reshape(3, 3).tolist(),
        'distortion_coefficients': DEFAULT_DIST.reshape(-1).tolist(),
        'width': int(args.width),
        'height': int(args.height),
        'device_id': args.device_id,
    }
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open('w', encoding='utf-8') as handle:
        json.dump(document, handle, indent=2)
        handle.write('\n')
    print(f'Saved calibration file: {out_path}')


if __name__ == '__main__':
    main()
