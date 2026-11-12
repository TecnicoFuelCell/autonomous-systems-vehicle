#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from webcam_pub.camera_exposure import (
    ExposureControlError,
    apply_exposure_settings,
    find_exposure_controls,
    measure_brightness,
    search_exposure,
    set_v4l2_control,
)


class WebcamPublisher(LifecycleNode):
    """Managed (lifecycle) camera publisher.

    configure:  open the camera, negotiate the capture mode and tune exposure
    activate:   start publishing frames
    deactivate: stop publishing (camera stays open)
    cleanup:    release the camera; a new configure re-tunes exposure

    Nothing is published until the node is active, so recordings never contain
    frames taken before the exposure was set for the current lighting.
    """

    def __init__(self):
        super().__init__('webcam_publisher')

        self.param_names = self.declare_node_parameters()

        self.bridge = CvBridge()
        self.cap = None
        self.publisher_ = None
        self.timer = None
        self.frame_count = 0

        self.get_logger().info('Waiting for the configure transition (launch autostart or ros2 lifecycle set)')

    # Lifecycle transitions

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        try:
            self.load_parameters()
            self.open_camera()
            self.configure_camera_exposure()
        except Exception as e:
            self.get_logger().error(f'Configure failed: {e}')
            self.release_camera()
            return TransitionCallbackReturn.FAILURE

        self.publisher_ = self.create_lifecycle_publisher(Image, self.topic_name, 10)
        self.get_logger().info(f'Configured; will publish to {self.topic_name} once activated')
        return super().on_configure(state)

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        result = super().on_activate(state)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.get_logger().info(f'Activated: publishing at {self.publish_rate} Hz')
        return result

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.destroy_publish_timer()
        self.get_logger().info('Deactivated: publishing stopped')
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        result = super().on_cleanup(state)
        self.release_resources()
        return result

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        result = super().on_shutdown(state)
        self.release_resources()
        return result

    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().error(f'Error while in state {state.label}; releasing the camera')
        self.release_resources()
        return TransitionCallbackReturn.SUCCESS

    # Configuration steps

    def load_parameters(self):
        self.device_path = self.get_parameter('camera_device_path').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.topic_name = self.get_parameter('camera_image_topic').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.camera_fps = self.get_parameter('camera_fps').value
        self.pixel_format = str(self.get_parameter('pixel_format').value).upper()

        resolved = '\n'.join(f'  {name}: {self.get_parameter(name).value}' for name in self.param_names)
        self.get_logger().info(f'Resolved parameters:\n{resolved}')

    def open_camera(self):
        if len(self.pixel_format) != 4:
            raise RuntimeError(f'pixel_format must be a 4-character code, got {self.pixel_format!r}')

        # Force the V4L2 backend on Jetson to avoid OpenCV/GStreamer auto-selection.
        self.cap = cv2.VideoCapture(self.device_path, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            self.get_logger().error('POSSIBLE ISSUE: If /dev/video0 is available inside the container, another process like rviz or cheese might be using the camera stream')
            raise RuntimeError(f'Cannot open camera device {self.device_path}')

        # Negotiate one of the formats explicitly advertised by the device.
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.pixel_format))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.camera_fps)

        negotiated_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        negotiated_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.negotiated_fps = self.cap.get(cv2.CAP_PROP_FPS)

        # Fail fast instead of activating a node that only emits repeated read warnings.
        ret, frame = self.cap.read()
        if not ret or frame is None:
            raise RuntimeError(
                f'Opened {self.device_path} but failed to read a frame. '
                f'Try a supported mode such as MJPG 1920x1080@60 or YUYV 1920x1080@30.'
            )

        self.get_logger().info(f'Camera opened successfully on device {self.device_path}')
        self.get_logger().info('Capture backend: V4L2')
        self.get_logger().info(f'Configured pixel format: {self.pixel_format}')
        self.get_logger().info(
            f'Negotiated capture mode: {negotiated_width}x{negotiated_height} @ {self.negotiated_fps:.1f} Hz'
        )
        # warn if the intrinsics topic's width x height don't match ours
        if negotiated_width != self.frame_width or negotiated_height != self.frame_height:
            self.get_logger().warn(f'WARNING: The capture resolution {negotiated_width}x{negotiated_height} that will be used \
            does not match the frame dimensions {self.frame_width}x{self.frame_height} in the intrisincs topic')

    def configure_camera_exposure(self):
        if not self.get_parameter('configure_exposure').value:
            self.get_logger().info('configure_exposure is false; leaving camera exposure untouched')
            return

        fail_fast = self.get_parameter('exposure_fail_fast').value
        exposure_absolute = self.get_parameter('exposure_absolute').value
        exposure_auto = self.get_parameter('exposure_auto').value

        controls, error = find_exposure_controls(self.device_path)
        if controls is None:
            self.report_exposure_problem(f'Cannot configure exposure: {error}', fail_fast)
            return
        self.get_logger().info(
            f'Exposure controls: {controls.auto_name}, {controls.absolute_name} '
            f'(device range {controls.absolute_min}..{controls.absolute_max})'
        )

        auto_tune = self.get_parameter('auto_tune_exposure').value
        if auto_tune:
            minimum, maximum = self.exposure_search_range(controls)
            exposure_absolute = min(max(exposure_absolute, minimum), maximum)

        # Puts the camera in manual mode and writes the fixed value, which is
        # also the starting point of the search below.
        applied = apply_exposure_settings(
            device_path=self.device_path,
            exposure_absolute=exposure_absolute,
            exposure_auto=exposure_auto,
            logger=self.get_logger(),
            fail_fast=fail_fast,
            controls=controls,
        )
        if not applied or not auto_tune:
            return

        try:
            self.tune_exposure(controls, exposure_absolute, minimum, maximum)
        except ExposureControlError as e:
            self.report_exposure_problem(f'Exposure tuning aborted: {e}', fail_fast)

    def tune_exposure(self, controls, initial_exposure, minimum, maximum):
        target = self.get_parameter('exposure_target_brightness').value
        tolerance = self.get_parameter('exposure_brightness_tolerance').value
        settle_frames = self.get_parameter('exposure_settle_frames').value
        top_fraction = self.get_parameter('exposure_metering_top_fraction').value

        self.get_logger().info(
            f'Tuning exposure in {minimum}..{maximum} (x100 us) '
            f'for brightness {target:.0f} +/- {tolerance:.0f}'
        )

        def measure(value):
            self.write_exposure(controls, value)
            # V4L2 queues several frames captured before the change; drop them
            # so the measured frame really uses the new exposure.
            for _ in range(settle_frames):
                self.cap.grab()
            ret, frame = self.cap.read()
            if not ret or frame is None:
                raise RuntimeError(f'Lost the camera stream on {self.device_path} while tuning exposure')
            brightness = measure_brightness(frame, top_fraction)
            self.get_logger().info(f'  exposure {value}: brightness {brightness:.1f}')
            return brightness

        result = search_exposure(
            measure,
            initial=initial_exposure,
            minimum=minimum,
            maximum=maximum,
            target=target,
            tolerance=tolerance,
            max_iterations=self.get_parameter('exposure_max_iterations').value,
        )

        self.write_exposure(controls, result.exposure)
        # Expose the chosen value through `ros2 param get` and use it as the
        # starting point if the node is configured again.
        self.set_parameters([Parameter('exposure_absolute', Parameter.Type.INTEGER, result.exposure)])

        if result.converged:
            self.get_logger().info(
                f'Exposure tuned to {result.exposure} (brightness {result.brightness:.1f}, '
                f'{result.iterations} steps)'
            )
        else:
            self.get_logger().warn(
                f'Exposure search did not reach brightness {target:.0f} +/- {tolerance:.0f} '
                f'in {result.iterations} steps; using closest value {result.exposure} '
                f'(brightness {result.brightness:.1f})'
            )

    def exposure_search_range(self, controls):
        minimum = self.get_parameter('exposure_min').value
        maximum = self.get_parameter('exposure_max').value
        if maximum <= 0:
            # Exposure is in 100 us units: one frame period is 10000 / fps.
            # Longer exposures lower the frame rate on UVC cameras.
            fps = self.negotiated_fps if self.negotiated_fps > 0 else self.camera_fps
            maximum = int(10000 / fps)

        if controls.absolute_min is not None:
            minimum = max(minimum, controls.absolute_min)
        if controls.absolute_max is not None:
            maximum = min(maximum, controls.absolute_max)
        if minimum > maximum:
            self.get_logger().warn(f'exposure_min {minimum} > exposure_max {maximum}; using {minimum} only')
            maximum = minimum
        return minimum, maximum

    def write_exposure(self, controls, value):
        ok, output = set_v4l2_control(self.device_path, controls.absolute_name, value)
        if not ok:
            raise ExposureControlError(f'Failed to set {controls.absolute_name}={value}: {output}')

    def report_exposure_problem(self, message, fail_fast):
        if fail_fast:
            raise RuntimeError(message)
        self.get_logger().warn(f'{message}; publishing with the current camera exposure')

    # Runtime

    def timer_callback(self):
        ret, frame = self.cap.read()

        if ret:
            # Convert OpenCV image to ROS Image message
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_frame'

                # Publish the message
                self.publisher_.publish(msg)

                self.frame_count += 1
                if self.frame_count % 300 == 0:
                    self.get_logger().info(f'Published {self.frame_count} frames')

            except Exception as e:
                self.get_logger().error(f'Error converting/publishing image: {str(e)}')
        else:
            self.get_logger().warn('Failed to capture frame from camera')

    # Resource handling

    def destroy_publish_timer(self):
        if self.timer is not None:
            self.destroy_timer(self.timer)
            self.timer = None

    def release_camera(self):
        if self.cap is not None:
            if self.cap.isOpened():
                self.cap.release()
                self.get_logger().info('Camera released')
            self.cap = None

    def release_resources(self):
        self.destroy_publish_timer()
        if self.publisher_ is not None:
            self.destroy_lifecycle_publisher(self.publisher_)
            self.publisher_ = None
        self.release_camera()

    def destroy_node(self):
        self.release_resources()
        super().destroy_node()

    def declare_node_parameters(self):
        # Declare parameters with defaults; override in vehicle_bringup/config/vehicle_params.yaml
        # (camera_device_path and camera_image_topic come from its shared /** section).
        # Exposure values are in V4L2 units of 100 us.
        self.declare_parameters(
            namespace='',
            parameters=[
                # Declare parameters
                ('camera_device_path', '/dev/video0'),
                ('publish_rate', 60.0),
                ('camera_image_topic', '/zed/left_camera/image_raw'),
                ('frame_width', 1280),
                ('frame_height', 720),
                ('camera_fps', 60.0),
                ('pixel_format', 'MJPG'),
                ('configure_exposure', True),
                ('exposure_absolute', 5),
                ('exposure_auto', 1),
                ('exposure_fail_fast', False),
                ('auto_tune_exposure', True),
                ('exposure_target_brightness', 110.0),
                ('exposure_brightness_tolerance', 15.0),
                ('exposure_min', 1),
                ('exposure_max', 0),
                ('exposure_max_iterations', 10),
                ('exposure_settle_frames', 8),
                ('exposure_metering_top_fraction', 0.0),
            ]
        )

        return ['camera_device_path', 'publish_rate', 'camera_image_topic', 'frame_width', 'frame_height',
                'camera_fps', 'pixel_format', 'configure_exposure', 'exposure_absolute',
                'exposure_auto', 'exposure_fail_fast', 'auto_tune_exposure',
                'exposure_target_brightness', 'exposure_brightness_tolerance', 'exposure_min',
                'exposure_max', 'exposure_max_iterations', 'exposure_settle_frames',
                'exposure_metering_top_fraction']


def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()

    try:
        rclpy.spin(webcam_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        webcam_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
