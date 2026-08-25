#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from commonsense.camera_exposure import apply_exposure_settings


class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        
        self.param_names = self.declare_node_parameters()

        # Get parameters
        device_path = self.get_parameter('device_path').value
        publish_rate = self.get_parameter('publish_rate').value
        topic_name = self.get_parameter('topic_name').value
        frame_width = self.get_parameter('frame_width').value
        frame_height = self.get_parameter('frame_height').value
        camera_fps = self.get_parameter('camera_fps').value
        pixel_format = str(self.get_parameter('pixel_format').value).upper()
        configure_exposure = self.get_parameter('configure_exposure').value
        exposure_absolute = self.get_parameter('exposure_absolute').value
        exposure_auto = self.get_parameter('exposure_auto').value
        exposure_fail_fast = self.get_parameter('exposure_fail_fast').value
        
        # debug log params
        self.get_logger().info(
            "Resolved parameters:\n" f"  device_path: {device_path}\n" f"  publish_rate: {publish_rate}\n"
            f"  topic_name: {topic_name}\n" f"  frame_width: {frame_width}\n" f"  frame_height: {frame_height}\n"
            f"  camera_fps: {camera_fps}\n" f"  pixel_format: {pixel_format}\n"
            f"  configure_exposure: {configure_exposure}\n"
            f"  exposure_absolute: {exposure_absolute}\n" f"  exposure_auto: {exposure_auto}"
        )

        # Create publisher
        self.publisher_ = self.create_publisher(Image, topic_name, 10)
        
        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Force the V4L2 backend on Jetson to avoid OpenCV/GStreamer auto-selection.
        self.cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera device {device_path}\n')
            self.get_logger().error('POSSIBLE ISSUE: If /dev/video0 is available inside the container, another process like rviz or cheese might be using the camera stream')
            raise RuntimeError(f'Cannot open camera device {device_path}')
        
        # Negotiate one of the formats explicitly advertised by the device.
        if len(pixel_format) != 4:
            raise RuntimeError(f'pixel_format must be a 4-character code, got {pixel_format!r}')

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*pixel_format))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, camera_fps)

        negotiated_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        negotiated_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        negotiated_fps = self.cap.get(cv2.CAP_PROP_FPS)

        # Fail fast instead of starting a node that only emits repeated read warnings.
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.cap.release()
            raise RuntimeError(
                f'Opened {device_path} but failed to read a frame. '
                f'Try a supported mode such as MJPG 1920x1080@60 or YUYV 1920x1080@30.'
            )

        if configure_exposure:
            apply_exposure_settings(
                device_path=device_path,
                exposure_absolute=exposure_absolute,
                exposure_auto=exposure_auto,
                logger=self.get_logger(),
                fail_fast=exposure_fail_fast,
            )
        
        self.get_logger().info(f'Camera opened successfully on device {device_path}')
        self.get_logger().info('Capture backend: V4L2')
        self.get_logger().info(f'Publishing to topic: {topic_name}')
        self.get_logger().info(f'Configured pixel format: {pixel_format}')
        self.get_logger().info(
            f'Negotiated capture mode: {negotiated_width}x{negotiated_height} @ {negotiated_fps:.1f} Hz'
        )
        self.get_logger().info(f'Publish rate: {publish_rate} Hz')
        # warn if the intrinsics topic's width x height don't match ours
        if negotiated_width != frame_width or negotiated_height != frame_height:
            self.get_logger().warn(f'WARNING: The capture resolution {negotiated_width}x{negotiated_height} that will be used \
            does not match the frame dimensions {frame_width}x{frame_height} in the intrisincs topic')
        
        # Create timer for publishing
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.frame_count = 0
    
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
    
    def destroy_node(self):
        # Release the camera when shutting down
        if self.cap.isOpened():
            self.cap.release()
            self.get_logger().info('Camera released')
        super().destroy_node()

    def declare_node_parameters(self):
        # Declare parameters with defaults (If you want to change this, change in car_params.yaml)
        self.declare_parameters(
            namespace='',
            parameters=[
                # Declare parameters
                ('device_path', '/dev/video0'),
                ('publish_rate', 60.0),
                ('topic_name', '/zed/left_camera/image_raw'),
                ('frame_width', 1280),
                ('frame_height', 720),
                ('camera_fps', 60.0),
                ('pixel_format', 'MJPG'),
                ('configure_exposure', True),
                ('exposure_absolute', 5),
                ('exposure_auto', 1),
                ('exposure_fail_fast', False),
            ]
        )

        return['device_path', 'publish_rate', 'topic_name', 'frame_width', 'frame_height',
               'camera_fps', 'pixel_format', 'configure_exposure', 'exposure_absolute',
               'exposure_auto', 'exposure_fail_fast']


def main(args=None):
    rclpy.init(args=args)
    
    try:
        webcam_publisher = WebcamPublisher()
        rclpy.spin(webcam_publisher)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
