#!/usr/bin/env python3
"""
Magnetometer calibration node for ROS2 Foxy.

Subscribes to /mag_data (sensor_msgs/MagneticField, default units: tesla),
accumulates samples, fits an ellipsoid (3D) or ellipse (2D / planar) to
estimate hard-iron offset and soft-iron transformation, and writes the
calibration to a YAML file.

Usage:
    ros2 run utils calibrate_magnetometer --output /path/to/calib.yaml
    ros2 run utils calibrate_magnetometer --output /path/to/calib.yaml --debug-window
    ros2 run utils calibrate_magnetometer --output /path/to/calib.yaml --planar
    ros2 run utils calibrate_magnetometer --output /path/to/calib.yaml --plot-output /path/to/plot.png
    ros2 run utils calibrate_magnetometer --output /path/to/calib.yaml --plot-topic /mag/calib_plot
    ros2 run utils calibrate_magnetometer --output /path/to/calib.yaml --plot-live-period 0.5

If --planar is omitted, the node auto-detects nearly-planar sample sets
(common for ground vehicles doing figure-8s) and falls back to 2D
calibration automatically.

Press Ctrl+C to stop sampling; calibration will be computed and saved.
"""

import argparse
import io
import sys
import threading
import time
from pathlib import Path

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, MagneticField


# ---------------------------------------------------------------------------
# Fitting routines
# ---------------------------------------------------------------------------

def fit_ellipsoid(samples: np.ndarray):
    """
    Fit a 3D ellipsoid to a set of points using the algebraic method
    (Li & Griffiths, 2004). Returns hard-iron offset (3,) and soft-iron
    correction matrix (3x3) such that:

        calibrated = soft_iron @ (raw - hard_iron)

    The soft-iron matrix maps the ellipsoid to a sphere whose radius equals
    the geometric mean of the ellipsoid's semi-axes.
    """
    if samples.shape[0] < 50:
        raise ValueError(
            f"Need at least ~50 samples for a stable 3D fit, got {samples.shape[0]}."
        )

    x = samples[:, 0]
    y = samples[:, 1]
    z = samples[:, 2]

    # Design matrix for general quadric:
    # a x^2 + b y^2 + c z^2 + 2f y z + 2g x z + 2h x y + 2p x + 2q y + 2r z = 1
    D = np.column_stack([
        x * x,
        y * y,
        z * z,
        2 * y * z,
        2 * x * z,
        2 * x * y,
        2 * x,
        2 * y,
        2 * z,
    ])
    ones = np.ones_like(x)

    v, *_ = np.linalg.lstsq(D, ones, rcond=None)
    a, b, c, f, g, h, p, q, r = v

    A4 = np.array([
        [a, h, g, p],
        [h, b, f, q],
        [g, f, c, r],
        [p, q, r, -1.0],
    ])
    A3 = A4[:3, :3]

    # Ellipsoid center (hard-iron offset)
    center = -np.linalg.solve(A3, np.array([p, q, r]))

    # Translate quadric to origin
    T = np.eye(4)
    T[3, :3] = center
    A4_centered = T @ A4 @ T.T

    scale = -A4_centered[3, 3]
    A3_centered = A4_centered[:3, :3] / scale

    eigvals, eigvecs = np.linalg.eigh(A3_centered)
    if np.any(eigvals <= 0):
        raise ValueError(
            "Ellipsoid fit produced non-positive eigenvalues; "
            "samples may not cover enough orientations."
        )

    radii = 1.0 / np.sqrt(eigvals)
    target_radius = float(np.cbrt(radii[0] * radii[1] * radii[2]))

    scale_matrix = np.diag(target_radius / radii)
    transform = eigvecs @ scale_matrix @ eigvecs.T

    return center, transform, target_radius


def fit_ellipse_2d(samples_xy: np.ndarray):
    """
    Fit a 2D ellipse to points in the XY plane. Returns center (2,)
    and a 2x2 transform mapping the ellipse to a circle whose radius
    equals the geometric mean of the semi-axes.

        calibrated_xy = transform @ (raw_xy - center)
    """
    if samples_xy.shape[0] < 20:
        raise ValueError(
            f"Need at least ~20 samples for 2D fit, got {samples_xy.shape[0]}."
        )

    x = samples_xy[:, 0]
    y = samples_xy[:, 1]

    # General conic: a x^2 + b x y + c y^2 + d x + e y = 1
    D = np.column_stack([x * x, x * y, y * y, x, y])
    ones = np.ones_like(x)
    v, *_ = np.linalg.lstsq(D, ones, rcond=None)
    a, b, c, d, e = v

    A2 = np.array([[a, b / 2.0],
                   [b / 2.0, c]])
    bias = np.array([d, e])

    try:
        center = -0.5 * np.linalg.solve(A2, bias)
    except np.linalg.LinAlgError:
        raise ValueError("2D ellipse fit is degenerate (samples nearly collinear).")

    # k can be negative for a valid off-origin ellipse because the least-squares
    # conic is fixed to "= 1" and may choose the opposite algebraic sign. The
    # real validity check is whether A2 / k is positive definite below.
    k = 1.0 + center @ A2 @ center
    if abs(k) <= np.finfo(float).eps:
        raise ValueError("2D ellipse fit produced near-zero scale.")

    A2_norm = A2 / k

    eigvals, eigvecs = np.linalg.eigh(A2_norm)
    if np.any(eigvals <= 0):
        raise ValueError(
            "2D ellipse fit produced non-positive eigenvalues; "
            "samples likely don't cover a full rotation."
        )

    radii = 1.0 / np.sqrt(eigvals)
    target_radius = float(np.sqrt(radii[0] * radii[1]))
    scale_matrix = np.diag(target_radius / radii)
    transform = eigvecs @ scale_matrix @ eigvecs.T

    return center, transform, target_radius


def fit_planar(samples: np.ndarray):
    """
    Planar calibration: fit a 2D ellipse in XY, apply mean offset on Z,
    and leave Z scale at unity. Suitable for ground vehicles where only
    yaw rotates significantly.
    """
    center_xy, transform_xy, field_strength = fit_ellipse_2d(samples[:, :2])
    z_offset = float(np.mean(samples[:, 2]))

    offset_3d = np.array([center_xy[0], center_xy[1], z_offset])
    transform_3d = np.eye(3)
    transform_3d[:2, :2] = transform_xy

    return offset_3d, transform_3d, field_strength


def planarity_ratio(samples: np.ndarray) -> float:
    """
    Return the ratio (smallest covariance eigenvalue) / (largest covariance
    eigenvalue). Values near 0 indicate planar (or worse, collinear) data.
    """
    if samples.shape[0] < 4:
        return 1.0
    cov = np.cov(samples.T)
    eigvals = np.linalg.eigvalsh(cov)
    largest = eigvals[-1]
    if largest <= 0:
        return 1.0
    return float(eigvals[0] / largest)


# ---------------------------------------------------------------------------
# Visualization
# ---------------------------------------------------------------------------

def unit_scale_to_microtesla(unit: str) -> float:
    normalized = unit.lower().replace("µ", "u")
    if normalized in ("t", "tesla", "teslas"):
        return 1.0e6
    if normalized in ("mt", "millitesla", "milliteslas"):
        return 1.0e3
    if normalized in ("ut", "microtesla", "microteslas"):
        return 1.0
    if normalized in ("nt", "nanotesla", "nanoteslas"):
        return 1.0e-3
    raise ValueError(
        f"Unsupported input unit {unit!r}. Use one of: t, mt, ut, nt."
    )


def apply_calibration(samples: np.ndarray, offset: np.ndarray,
                      transform: np.ndarray) -> np.ndarray:
    """Apply column-vector calibration formula to row-wise sample arrays."""
    return (samples - offset) @ transform.T


def render_xy_plot_png(raw_samples: np.ndarray, calibrated_samples: np.ndarray,
                       units: str) -> bytes:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(11, 5), constrained_layout=True)
    for ax, title, samples in (
        (axes[0], "Raw magnetometer XY", raw_samples),
        (axes[1], "Calibrated magnetometer XY", calibrated_samples),
    ):
        ax.scatter(samples[:, 0], samples[:, 1], s=6, alpha=0.7)
        ax.axhline(0.0, color="0.75", linewidth=0.8)
        ax.axvline(0.0, color="0.75", linewidth=0.8)
        ax.set_title(title)
        ax.set_xlabel(f"X ({units})")
        ax.set_ylabel(f"Y ({units})")
        ax.grid(True, alpha=0.25)
        ax.set_aspect("equal", adjustable="box")

    all_xy = np.vstack([raw_samples[:, :2], calibrated_samples[:, :2]])
    mins = all_xy.min(axis=0)
    maxs = all_xy.max(axis=0)
    span = max(float(np.max(maxs - mins)), 1.0e-9)
    center = 0.5 * (mins + maxs)
    margin = 0.55 * span
    for ax in axes:
        ax.set_xlim(center[0] - margin, center[0] + margin)
        ax.set_ylim(center[1] - margin, center[1] + margin)

    buffer = io.BytesIO()
    fig.savefig(buffer, format="png", dpi=160)
    plt.close(fig)
    return buffer.getvalue()


def _set_equal_2d_limits(ax, first: np.ndarray, second: np.ndarray):
    points = np.column_stack([first, second])
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    span = max(float(np.max(maxs - mins)), 1.0e-9)
    center = 0.5 * (mins + maxs)
    margin = 0.55 * span
    ax.set_xlim(center[0] - margin, center[0] + margin)
    ax.set_ylim(center[1] - margin, center[1] + margin)


def render_live_plot_png(samples: np.ndarray, units: str) -> bytes:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    fig = plt.figure(figsize=(11, 8), constrained_layout=True)
    ax3d = fig.add_subplot(2, 2, 1, projection="3d")
    ax_xy = fig.add_subplot(2, 2, 2)
    ax_xz = fig.add_subplot(2, 2, 3)
    ax_yz = fig.add_subplot(2, 2, 4)

    fig.suptitle(f"Live magnetometer samples ({samples.shape[0]} points)")

    ax3d.scatter(samples[:, 0], samples[:, 1], samples[:, 2], s=5, alpha=0.75)
    ax3d.set_title("Raw 3D")
    ax3d.set_xlabel(f"X ({units})")
    ax3d.set_ylabel(f"Y ({units})")
    ax3d.set_zlabel(f"Z ({units})")

    mins = samples.min(axis=0)
    maxs = samples.max(axis=0)
    span = max(float(np.max(maxs - mins)), 1.0e-9)
    center = 0.5 * (mins + maxs)
    margin = 0.55 * span
    ax3d.set_xlim(center[0] - margin, center[0] + margin)
    ax3d.set_ylim(center[1] - margin, center[1] + margin)
    ax3d.set_zlim(center[2] - margin, center[2] + margin)

    projection_specs = (
        (ax_xy, "XY / heading circle", 0, 1, "X", "Y"),
        (ax_xz, "XZ / pitch coverage", 0, 2, "X", "Z"),
        (ax_yz, "YZ / roll coverage", 1, 2, "Y", "Z"),
    )
    for ax, title, first_idx, second_idx, first_name, second_name in projection_specs:
        ax.scatter(samples[:, first_idx], samples[:, second_idx], s=6, alpha=0.75)
        ax.axhline(0.0, color="0.75", linewidth=0.8)
        ax.axvline(0.0, color="0.75", linewidth=0.8)
        ax.set_title(title)
        ax.set_xlabel(f"{first_name} ({units})")
        ax.set_ylabel(f"{second_name} ({units})")
        ax.grid(True, alpha=0.25)
        ax.set_aspect("equal", adjustable="box")
        _set_equal_2d_limits(ax, samples[:, first_idx], samples[:, second_idx])

    buffer = io.BytesIO()
    fig.savefig(buffer, format="png", dpi=140)
    plt.close(fig)
    return buffer.getvalue()


def save_xy_plot(png_bytes: bytes, output_path: str):
    output = Path(output_path).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(png_bytes)
    return output


def save_samples_csv(samples: np.ndarray, output_path: str, units: str):
    output = Path(output_path).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    header = f"x_{units},y_{units},z_{units}"
    np.savetxt(output, samples, delimiter=",", header=header, comments="")
    return output


def save_artifacts(node, args, samples: np.ndarray, offset: np.ndarray,
                   transform: np.ndarray):
    if args.samples_output:
        samples_path = save_samples_csv(samples, args.samples_output, "microtesla")
        node.get_logger().info(f"Wrote raw samples to {samples_path}")

    if args.plot_output or args.plot_topic:
        calibrated = apply_calibration(samples, offset, transform)
        png_bytes = render_xy_plot_png(samples, calibrated, "microtesla")
        if args.plot_output:
            plot_path = save_xy_plot(png_bytes, args.plot_output)
            node.get_logger().info(f"Wrote before/after XY plot to {plot_path}")
        if args.plot_topic:
            node.publish_plot_png(png_bytes, args.plot_publish_seconds)


class MagVisualizer:
    """Live 3D scatter plot of raw magnetometer samples."""

    def __init__(self):
        import matplotlib
        matplotlib.use("TkAgg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

        self.plt = plt
        plt.ion()
        self.fig = plt.figure("Magnetometer Calibration (raw, µT)")
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.scatter = self.ax.scatter([], [], [], s=2, c="tab:blue")
        self.ax.set_xlabel("X (µT)")
        self.ax.set_ylabel("Y (µT)")
        self.ax.set_zlabel("Z (µT)")
        self.ax.set_title("Rotate the sensor through all orientations")
        self.fig.tight_layout()
        self._last_drawn = 0

    def update(self, samples: np.ndarray):
        if samples.shape[0] == self._last_drawn:
            self.fig.canvas.flush_events()
            return
        self._last_drawn = samples.shape[0]

        self.scatter._offsets3d = (samples[:, 0], samples[:, 1], samples[:, 2])

        mins = samples.min(axis=0)
        maxs = samples.max(axis=0)
        margin = 0.1 * (maxs - mins + 1e-6)
        self.ax.set_xlim(mins[0] - margin[0], maxs[0] + margin[0])
        self.ax.set_ylim(mins[1] - margin[1], maxs[1] + margin[1])
        self.ax.set_zlim(mins[2] - margin[2], maxs[2] + margin[2])

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def close(self):
        self.plt.close(self.fig)


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

class MagCalibrationNode(Node):
    def __init__(self, output_path: str, debug_window: bool, planar: bool,
                 input_units: str, plot_topic: str, plot_live_period: float):
        super().__init__("mag_calibration_node")

        self.output_path = Path(output_path).expanduser().resolve()
        self.debug_window = debug_window
        self.planar = planar
        self.input_units = input_units
        self.input_scale = unit_scale_to_microtesla(input_units)
        self.plot_topic = plot_topic
        self.plot_live_period = plot_live_period
        self._last_live_plot_sample_count = 0

        self._samples = []
        self._lock = threading.Lock()

        self.subscription = self.create_subscription(
            MagneticField,
            "/mag_data",
            self._mag_callback,
            qos_profile=10,
        )

        self.plot_publisher = None
        if plot_topic:
            plot_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.plot_publisher = self.create_publisher(
                CompressedImage,
                plot_topic,
                plot_qos,
            )

        mode_str = "planar (2D)" if planar else "auto (3D with planar fallback)"
        self.get_logger().info(
            f"Listening on /mag_data. Mode: {mode_str}. Output YAML: {self.output_path}"
        )
        self.get_logger().info(
            f"Interpreting incoming MagneticField values as {input_units}; "
            "calibration is stored in microtesla."
        )
        self.get_logger().info(
            "Rotate the sensor / drive the vehicle through as many headings as "
            "possible. Press Ctrl+C to finish."
        )

        self.create_timer(2.0, self._status_tick)
        if self.plot_publisher is not None and plot_live_period > 0.0:
            self.create_timer(plot_live_period, self._publish_live_plot)

    def _mag_callback(self, msg: MagneticField):
        sample = np.array(
            [msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z],
            dtype=np.float64,
        ) * self.input_scale
        with self._lock:
            self._samples.append(sample)

    def _status_tick(self):
        with self._lock:
            n = len(self._samples)
        self.get_logger().info(f"Collected {n} samples...")

    def get_samples(self) -> np.ndarray:
        with self._lock:
            if not self._samples:
                return np.empty((0, 3))
            return np.vstack(self._samples)

    def _publish_live_plot(self):
        samples = self.get_samples()
        n_samples = samples.shape[0]
        if n_samples < 2 or n_samples == self._last_live_plot_sample_count:
            return

        self._last_live_plot_sample_count = n_samples
        try:
            self.publish_plot_once(render_live_plot_png(samples, "microtesla"))
        except Exception as e:
            self.get_logger().warn(f"Could not publish live calibration plot: {e}")

    def save_calibration(self, offset, transform, field_strength,
                         n_samples, mode):
        data = {
            "mag_calibration": {
                "frame_id": "mag",
                "input_units": self.input_units,
                "units": "microtesla",
                "mode": mode,
                "n_samples": int(n_samples),
                "expected_field_strength": float(field_strength),
                "hard_iron_offset": [float(v) for v in offset],
                "soft_iron_matrix": [
                    [float(v) for v in row] for row in transform
                ],
                "usage": (
                    "calibrated = soft_iron_matrix @ (raw - hard_iron_offset)"
                ),
            }
        }
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self.output_path, "w") as f:
            yaml.safe_dump(data, f, sort_keys=False, default_flow_style=None)
        self.get_logger().info(f"Wrote calibration to {self.output_path}")

    def publish_plot_png(self, png_bytes: bytes, publish_seconds: float):
        if self.plot_publisher is None:
            return

        deadline = time.monotonic() + max(0.0, publish_seconds)
        published = 0
        while time.monotonic() <= deadline or published == 0:
            self.publish_plot_once(png_bytes)
            published += 1
            time.sleep(0.25)

        self.get_logger().info(
            f"Published before/after calibration PNG to {self.plot_topic} "
            f"({published} message(s))"
        )

    def publish_plot_once(self, png_bytes: bytes):
        if self.plot_publisher is None:
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "mag_calibration_plot"
        msg.format = "png"
        msg.data = png_bytes
        self.plot_publisher.publish(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Magnetometer hard/soft iron calibration node."
    )
    parser.add_argument(
        "--output", "-o",
        required=True,
        help="Path to the YAML file where calibration will be written.",
    )
    parser.add_argument(
        "--debug-window",
        action="store_true",
        help="Open a live 3D plot of incoming samples.",
    )
    parser.add_argument(
        "--input-units",
        default="t",
        choices=("t", "tesla", "mt", "millitesla", "ut", "microtesla",
                 "nt", "nanotesla"),
        help="Units used by incoming /mag_data values. Default: t, matching "
             "sensor_msgs/MagneticField.",
    )
    parser.add_argument(
        "--plot-output",
        help="Optional path to save a before/after XY calibration plot as PNG.",
    )
    parser.add_argument(
        "--plot-topic",
        default="/mag/calib_plot",
        help="Topic for publishing live and final matplotlib PNGs as "
             "sensor_msgs/msg/CompressedImage. Use an empty string to disable. "
             "Default: /mag/calib_plot.",
    )
    parser.add_argument(
        "--plot-publish-seconds",
        type=float,
        default=5.0,
        help="How long to keep publishing the final calibration plot after "
             "calibration finishes. Default: 5.0.",
    )
    parser.add_argument(
        "--plot-live-period",
        type=float,
        default=1.0,
        help="Seconds between live matplotlib PNG updates on --plot-topic "
             "while samples are being collected. Set <= 0 to disable live "
             "updates. Default: 1.0.",
    )
    parser.add_argument(
        "--samples-output",
        help="Optional path to save collected raw samples as CSV in microtesla.",
    )
    parser.add_argument(
        "--planar",
        action="store_true",
        help="Force 2D ellipse fit in the XY plane (use for ground vehicles "
             "that mostly rotate in yaw). Otherwise the node tries 3D first "
             "and falls back to 2D if the data is nearly planar.",
    )
    parser.add_argument(
        "--planarity-threshold",
        type=float,
        default=0.01,
        help="If (smallest/largest) covariance eigenvalue is below this, "
             "auto-fallback to planar mode. Default: 0.01.",
    )
    return parser.parse_known_args(argv)


def main():
    args, ros_args = parse_args(sys.argv[1:])

    rclpy.init(args=ros_args)
    node = MagCalibrationNode(
        args.output,
        args.debug_window,
        args.planar,
        args.input_units,
        args.plot_topic,
        args.plot_live_period,
    )

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    visualizer = None
    if args.debug_window:
        try:
            visualizer = MagVisualizer()
        except Exception as e:
            node.get_logger().error(f"Could not open debug window: {e}")
            visualizer = None

    try:
        if visualizer is not None:
            while rclpy.ok():
                samples = node.get_samples()
                if samples.shape[0] > 0:
                    visualizer.update(samples)
                else:
                    visualizer.fig.canvas.flush_events()
                visualizer.plt.pause(0.05)
        else:
            spin_thread.join()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted, computing calibration...")

    executor.shutdown()

    samples = node.get_samples()
    node.get_logger().info(f"Total samples collected: {samples.shape[0]}")

    # Decide which fit to use
    mode_used = None
    try:
        if samples.shape[0] == 0:
            raise ValueError("No samples were collected.")

        flatness = planarity_ratio(samples)
        node.get_logger().info(f"Sample planarity ratio: {flatness:.2e} "
                               f"(threshold: {args.planarity_threshold:.2e})")

        use_planar = args.planar or (flatness < args.planarity_threshold)

        if use_planar:
            if not args.planar:
                node.get_logger().warn(
                    "Samples are nearly planar; falling back to 2D calibration."
                )
            node.get_logger().info("Using planar (2D) calibration mode.")
            offset, transform, field_strength = fit_planar(samples)
            mode_used = "planar"
        else:
            node.get_logger().info("Using 3D ellipsoid calibration mode.")
            offset, transform, field_strength = fit_ellipsoid(samples)
            mode_used = "ellipsoid"

        node.get_logger().info(f"Hard-iron offset (µT): {offset}")
        node.get_logger().info(f"Field strength (µT): {field_strength:.3f}")
        node.get_logger().info(f"Soft-iron matrix:\n{transform}")
        node.save_calibration(offset, transform, field_strength,
                              samples.shape[0], mode_used)
        save_artifacts(node, args, samples, offset, transform)
    except Exception as e:
        node.get_logger().error(f"Calibration failed: {e}")
        # As a last resort, if a 3D fit failed but planar wasn't tried, try it.
        if mode_used is None and samples.shape[0] >= 20 and not args.planar:
            node.get_logger().warn(
                "Retrying with planar (2D) calibration as a last resort..."
            )
            try:
                offset, transform, field_strength = fit_planar(samples)
                node.get_logger().info(f"Hard-iron offset (µT): {offset}")
                node.get_logger().info(f"Field strength (µT): {field_strength:.3f}")
                node.get_logger().info(f"Soft-iron matrix:\n{transform}")
                node.save_calibration(offset, transform, field_strength,
                                      samples.shape[0], "planar")
                save_artifacts(node, args, samples, offset, transform)
            except Exception as e2:
                node.get_logger().error(f"Planar fallback also failed: {e2}")

    if visualizer is not None:
        visualizer.close()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
