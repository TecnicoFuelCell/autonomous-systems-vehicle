#!/usr/bin/env python3

import argparse
import math
import re
import subprocess
from typing import Callable, NamedTuple, Optional

import cv2

# Linux 5.x renamed the UVC exposure controls; accept both so the same code
# works on older and newer kernels. The first name found on the device wins.
EXPOSURE_AUTO_NAMES = ("auto_exposure", "exposure_auto")
EXPOSURE_ABSOLUTE_NAMES = ("exposure_time_absolute", "exposure_absolute")

# One line of `v4l2-ctl --list-ctrls`, e.g.
#   exposure_time_absolute 0x009a0902 (int)    : min=3 max=2047 step=1 default=250 value=250
_CONTROL_LINE = re.compile(r"^\s*(\w+)\s+0x[0-9a-fA-F]+\s+\((\w+)\)\s*:\s*(.*)$")
_CONTROL_FIELD = re.compile(r"(\w+)=(-?\d+)")


class ExposureControlError(RuntimeError):
    """The camera rejected or does not support an exposure control write."""


class ExposureControls(NamedTuple):
    auto_name: str
    absolute_name: str
    # Exposure range reported by the driver, in 100 us units (None if unknown).
    absolute_min: Optional[int]
    absolute_max: Optional[int]


class ExposureSearchResult(NamedTuple):
    exposure: int
    brightness: float
    converged: bool
    iterations: int


def _run_v4l2_ctl(device_path, *args):
    """Run v4l2-ctl and return (ok, output_or_error_text)."""
    try:
        result = subprocess.run(
            ["v4l2-ctl", "-d", device_path, *args],
            capture_output=True,
            text=True,
            check=False,
        )
    except FileNotFoundError:
        return False, "v4l2-ctl not found (install the v4l-utils package)"
    if result.returncode != 0:
        return False, result.stderr.strip() or result.stdout.strip() or "unknown v4l2-ctl error"
    return True, result.stdout.strip()


def set_v4l2_control(device_path, control_name, value):
    return _run_v4l2_ctl(device_path, f"--set-ctrl={control_name}={value}")


def get_v4l2_control(device_path, control_name):
    return _run_v4l2_ctl(device_path, f"--get-ctrl={control_name}")


def parse_v4l2_controls(text):
    """Parse `v4l2-ctl --list-ctrls` output into {name: {field: int}}."""
    controls = {}
    for line in text.splitlines():
        match = _CONTROL_LINE.match(line)
        if match is None:
            continue
        name, _, fields = match.groups()
        controls[name] = {key: int(value) for key, value in _CONTROL_FIELD.findall(fields)}
    return controls


def find_exposure_controls(device_path):
    """Detect the exposure control names and range. Returns (controls, error)."""
    ok, output = _run_v4l2_ctl(device_path, "--list-ctrls")
    if not ok:
        return None, output

    controls = parse_v4l2_controls(output)
    auto_name = next((name for name in EXPOSURE_AUTO_NAMES if name in controls), None)
    absolute_name = next((name for name in EXPOSURE_ABSOLUTE_NAMES if name in controls), None)
    if auto_name is None or absolute_name is None:
        return None, f"no manual exposure controls found on {device_path} (controls: {sorted(controls)})"

    absolute = controls[absolute_name]
    return ExposureControls(auto_name, absolute_name, absolute.get("min"), absolute.get("max")), None


def apply_exposure_settings(
    device_path="/dev/video0",
    exposure_absolute=5,
    exposure_auto=1,
    logger=None,
    fail_fast=False,
    controls=None,
):
    """Switch the camera to manual exposure and write a fixed exposure value."""

    def report(message, is_error):
        if logger is not None:
            logger.error(message) if is_error else logger.warn(message)
        if is_error:
            raise RuntimeError(message)

    if controls is None:
        controls, error = find_exposure_controls(device_path)
        if controls is None:
            report(f"Cannot configure exposure: {error}", fail_fast)
            return False

    # The auto mode must be written first: UVC drivers reject the absolute
    # exposure while auto exposure is still enabled.
    commands = [
        (controls.auto_name, exposure_auto),
        (controls.absolute_name, exposure_absolute),
    ]

    applied = True
    for control_name, value in commands:
        ok, output = set_v4l2_control(device_path, control_name, value)
        if not ok:
            report(f"Failed to set {control_name}={value} on {device_path}: {output}", fail_fast)
            applied = False
            continue
        if logger is not None:
            logger.info(f"Set {control_name}={value} on {device_path}")

    for control_name, _ in commands:
        ok, output = get_v4l2_control(device_path, control_name)
        if logger is None:
            continue
        if ok:
            logger.info(f"Verified {output}")
        else:
            logger.warn(f"Could not verify {control_name} on {device_path}: {output}")

    return applied


def measure_brightness(frame, top_fraction=0.0):
    """Mean grayscale value (0-255) of a BGR frame.

    top_fraction drops the top part of the image (e.g. sky) so the exposure is
    metered on the road. The frame is downscaled first; the mean barely changes
    and it is much cheaper on full-HD frames.
    """
    height = frame.shape[0]
    start_row = min(int(height * max(0.0, top_fraction)), height - 1)
    region = frame[start_row:, :]
    small = cv2.resize(region, (160, max(1, int(160 * region.shape[0] / region.shape[1]))),
                       interpolation=cv2.INTER_AREA)
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    return float(gray.mean())


def search_exposure(
    measure: Callable[[int], float],
    initial: int,
    minimum: int,
    maximum: int,
    target: float,
    tolerance: float,
    max_iterations: int,
):
    """Find an exposure whose brightness is within target +/- tolerance.

    Brightness grows with exposure, so this is a binary search. The midpoint is
    geometric because brightness scales with the exposure ratio (doubling the
    exposure roughly doubles the light), not the difference. If no value hits
    the band, the closest one seen is returned with converged=False.
    """
    low, high = minimum, maximum
    value = min(max(initial, low), high)
    best = None

    for iteration in range(1, max(1, max_iterations) + 1):
        brightness = measure(value)
        if best is None or abs(brightness - target) < abs(best.brightness - target):
            best = ExposureSearchResult(value, brightness, False, iteration)

        if abs(brightness - target) <= tolerance:
            return ExposureSearchResult(value, brightness, True, iteration)

        if brightness < target:
            low = value + 1
        else:
            high = value - 1
        if low > high:
            break
        value = int(round(math.sqrt(max(low, 1) * max(high, 1))))
        value = min(max(value, low), high)

    return best._replace(iterations=iteration)


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
