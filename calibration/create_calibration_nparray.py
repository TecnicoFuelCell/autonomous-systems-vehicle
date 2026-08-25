import argparse
from pathlib import Path

import numpy as np

# CHANGE THESE TWO BELOW
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

def create_npz(file_name, mtx, dist):
    if mtx.shape != (3, 3):
        raise ValueError(f"Camera matrix must be 3x3, got {mtx.shape}")
    if dist.shape not in {(1, 5), (5,), (1, 8), (8,)}:
        raise ValueError(
            "Distortion coefficients must have 5 or 8 values "
            f"(shape (1,5)/(5,) or (1,8)/(8,)), got {dist.shape}"
        )

    dist_out = dist.reshape(1, -1)
    output_path = Path(file_name)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(output_path, mtx=mtx, dist=dist_out)
    return output_path


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=str,
        default="default.npz",
        help="Output .npz path (default: calibration.npz)",
    )
    return parser


def main():
    parser = parse_args()
    args = parser.parse_args()
    out_path = create_npz(args.output, MATRIX, DEFAULT_DIST)
    print(f"Saved calibration file: {out_path}")
    print("Keys:")
    print("  mtx shape:", MATRIX.shape)
    print("  dist shape:", DEFAULT_DIST.shape)


if __name__ == "__main__":
    main()