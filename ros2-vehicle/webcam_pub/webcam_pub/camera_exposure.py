#!/usr/bin/env python3

import argparse
import subprocess


def set_v4l2_control(device_path, control_name, value):
    return subprocess.run(
        [
            "v4l2-ctl",
            "-d",
            device_path,
            f"--set-ctrl={control_name}={value}",
        ],
        capture_output=True,
        text=True,
        check=False,
    )


def get_v4l2_control(device_path, control_name):
    return subprocess.run(
        [
            "v4l2-ctl",
            "-d",
            device_path,
            f"--get-ctrl={control_name}",
        ],
        capture_output=True,
        text=True,
        check=False,
    )


def apply_exposure_settings(
    device_path="/dev/video0",
    exposure_absolute=5,
    exposure_auto=1,
    logger=None,
    fail_fast=False,
):
    commands = [
        ("exposure_absolute", exposure_absolute),
        ("exposure_auto", exposure_auto),
    ]

    applied = True
    for control_name, value in commands:
        result = set_v4l2_control(device_path, control_name, value)
        if result.returncode != 0:
            message = (
                f"Failed to set {control_name}={value} on {device_path}: "
                f"{result.stderr.strip() or result.stdout.strip() or 'unknown v4l2-ctl error'}"
            )
            if logger is not None:
                logger.error(message) if fail_fast else logger.warn(message)
            if fail_fast:
                raise RuntimeError(message)
            applied = False
            continue

        if logger is not None:
            logger.info(f"Set {control_name}={value} on {device_path}")

    for control_name, _ in commands:
        result = get_v4l2_control(device_path, control_name)
        if result.returncode == 0:
            if logger is not None:
                logger.info(f"Verified {result.stdout.strip()}")
        elif logger is not None:
            logger.warn(
                f"Could not verify {control_name} on {device_path}: "
                f"{result.stderr.strip() or result.stdout.strip() or 'unknown v4l2-ctl error'}"
            )

    return applied


def main():
    parser = argparse.ArgumentParser(description="Apply fixed V4L2 camera exposure settings.")
    parser.add_argument("-d", "--device", default="/dev/video0")
    parser.add_argument("--exposure-absolute", type=int, default=5)
    parser.add_argument("--exposure-auto", type=int, default=1)
    args = parser.parse_args()

    ok = apply_exposure_settings(
        device_path=args.device,
        exposure_absolute=args.exposure_absolute,
        exposure_auto=args.exposure_auto,
        fail_fast=False,
    )
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
