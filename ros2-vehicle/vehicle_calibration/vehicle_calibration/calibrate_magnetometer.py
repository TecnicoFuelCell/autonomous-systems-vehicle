#!/usr/bin/env python3
"""
Magnetometer hard/soft-iron calibration node.

Subscribes to the raw magnetometer topic (sensor_msgs/MagneticField), collects
samples while the car turns through all headings, and on Ctrl+C fits an
ellipse (planar, 2D) or ellipsoid (3D) and writes the correction to
mag_calib_path as JSON:

    calibrated = soft_iron_matrix @ (raw - hard_iron_offset)

mag_node must publish uncorrected data (apply_mag_calib=false) while sampling;
launch/calibrate_magnetometer.launch.xml sets this up. Topics and output path
come from the shared section of vehicle_params.yaml; the other parameters
from config/calibration_params.yaml.

Outputs:
    mag_calib_path   calibration JSON (always)
    magnetometer_plot_topic  live sample plot while sampling (view with rqt_image_view)
    <mag_calib_path>.png  before/after XY plot, only when save_plot is true
"""

import io
import json
from pathlib import Path

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, MagneticField

# Fallbacks when the node is started without launch params.
_DEFAULT_MAG_TOPIC = '/mag_data'
_DEFAULT_MAG_CALIB_PATH = 'calibration/mag_calib.json'
_DEFAULT_MAG_PLOT_TOPIC = '/vehicle_internal/calibration/magnetometer/plot'

# sensor_msgs/MagneticField is in tesla; the calibration is stored in microtesla.
_TESLA_TO_MICROTESLA = 1.0e6
_LIVE_PLOT_PERIOD_SEC = 1.0

# Latched so a viewer started late still receives the latest plot.
PLOT_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


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
    for ax in axes:
        _set_equal_2d_limits(ax, all_xy[:, 0], all_xy[:, 1])

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


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

class MagCalibrator(Node):
    def __init__(self):
        super().__init__("calibrate_magnetometer")

        self.declare_parameter("mag_topic", _DEFAULT_MAG_TOPIC)
        self.declare_parameter("mag_calib_path", _DEFAULT_MAG_CALIB_PATH)
        self.declare_parameter("magnetometer_plot_topic", _DEFAULT_MAG_PLOT_TOPIC)
        self.declare_parameter("planar", True)
        self.declare_parameter("planarity_threshold", 0.01)
        self.declare_parameter("save_plot", False)

        mag_topic = str(self.get_parameter("mag_topic").value)
        self.output_path = Path(
            str(self.get_parameter("mag_calib_path").value)
        ).expanduser().resolve()
        plot_topic = str(self.get_parameter("magnetometer_plot_topic").value).strip()
        self.planar = bool(self.get_parameter("planar").value)
        self.planarity_threshold = float(self.get_parameter("planarity_threshold").value)
        self.save_plot = bool(self.get_parameter("save_plot").value)

        self._samples = []
        self._last_live_plot_sample_count = 0

        self.subscription = self.create_subscription(
            MagneticField, mag_topic, self.mag_callback, 10,
        )

        self.plot_publisher = None
        if plot_topic:
            self.plot_publisher = self.create_publisher(
                CompressedImage, plot_topic, PLOT_QOS,
            )
            self.create_timer(_LIVE_PLOT_PERIOD_SEC, self.publish_live_plot)
        self.create_timer(2.0, self.log_status)

        mode_str = "planar (2D)" if self.planar else "auto (3D with planar fallback)"
        self.get_logger().info(
            f"Listening on {mag_topic}. Mode: {mode_str}. Output: {self.output_path}"
        )
        if plot_topic:
            self.get_logger().info(f"Live sample plot on {plot_topic}")
        self.get_logger().info(
            "Drive the vehicle slowly through as many headings as possible "
            "(level ground). Press Ctrl+C to finish."
        )

    def mag_callback(self, msg: MagneticField):
        self._samples.append([
            msg.magnetic_field.x * _TESLA_TO_MICROTESLA,
            msg.magnetic_field.y * _TESLA_TO_MICROTESLA,
            msg.magnetic_field.z * _TESLA_TO_MICROTESLA,
        ])

    def log_status(self):
        self.get_logger().info(f"Collected {len(self._samples)} samples...")

    def get_samples(self) -> np.ndarray:
        if not self._samples:
            return np.empty((0, 3))
        return np.asarray(self._samples, dtype=np.float64)

    def publish_live_plot(self):
        samples = self.get_samples()
        n_samples = samples.shape[0]
        if n_samples < 2 or n_samples == self._last_live_plot_sample_count:
            return

        self._last_live_plot_sample_count = n_samples
        try:
            self.publish_plot_once(render_live_plot_png(samples, "microtesla"))
        except Exception as e:
            self.get_logger().warn(f"Could not publish live calibration plot: {e}")

    def compute_calibration(self, samples: np.ndarray):
        """Return (offset, transform, field_strength, mode); raises on failure."""
        if samples.shape[0] == 0:
            raise ValueError("No samples were collected.")

        if not self.planar:
            flatness = planarity_ratio(samples)
            self.get_logger().info(
                f"Sample planarity ratio: {flatness:.2e} "
                f"(threshold: {self.planarity_threshold:.2e})"
            )
            if flatness < self.planarity_threshold:
                self.get_logger().warn(
                    "Samples are nearly planar; using planar (2D) calibration."
                )
            else:
                try:
                    return (*fit_ellipsoid(samples), "ellipsoid")
                except (ValueError, np.linalg.LinAlgError) as e:
                    self.get_logger().warn(
                        f"3D ellipsoid fit failed ({e}); falling back to planar (2D)."
                    )

        return (*fit_planar(samples), "planar")

    def finalize(self) -> bool:
        samples = self.get_samples()
        self.get_logger().info(f"Total samples collected: {samples.shape[0]}")
        try:
            offset, transform, field_strength, mode = self.compute_calibration(samples)
        except (ValueError, np.linalg.LinAlgError) as e:
            self.get_logger().error(f"Calibration failed: {e}")
            return False

        self.get_logger().info(f"Mode: {mode}")
        self.get_logger().info(f"Hard-iron offset (µT): {offset}")
        self.get_logger().info(f"Field strength (µT): {field_strength:.3f}")
        self.get_logger().info(f"Soft-iron matrix:\n{transform}")
        self.save_calibration(offset, transform, field_strength, samples.shape[0], mode)
        if self.save_plot:
            self.save_result_plot(samples, offset, transform)
        return True

    def save_result_plot(self, samples: np.ndarray, offset: np.ndarray,
                         transform: np.ndarray):
        calibrated = apply_calibration(samples, offset, transform)
        path = self.output_path.with_suffix(".png")
        path.write_bytes(render_xy_plot_png(samples, calibrated, "microtesla"))
        self.get_logger().info(f"Wrote before/after XY plot to {path}")

    def save_calibration(self, offset, transform, field_strength,
                         n_samples, mode):
        mag_data = {
            "frame_id": "mag",
            "input_units": "t",
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
        path = Path(self.output_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('w', encoding='utf-8') as handle:
            json.dump(mag_data, handle, indent=2)
            handle.write('\n')
        self.get_logger().info(f"Wrote calibration to {path}")

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

def main(args=None):
    # Ctrl+C ends sampling; the fit and file writes run afterwards and need no ROS.
    rclpy.init(args=args)
    node = MagCalibrator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.get_logger().info("Sampling stopped, computing calibration...")
        try:
            node.finalize()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
