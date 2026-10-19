import json
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

_DEFAULT_EXTRINSICS_PATH = 'calibration/camera_extrinsics.json'
_DEFAULT_INTRINSICS_PATH = 'calibration/camera_intrinsics.json'


class ExtrinsicCalibrator(Node):
    def __init__(self):
        super().__init__('calibrate_extrinsics')
        self.declare_parameter('camera_image_topic', '/zed/left_camera/image_raw')
        self.declare_parameter('extrinsics_image_topic', '/vehicle_internal/calibration/extrinsics/image')
        self.declare_parameter('camera_intrinsics_path', _DEFAULT_INTRINSICS_PATH)
        self.declare_parameter('camera_extrinsics_path', _DEFAULT_EXTRINSICS_PATH)
        self.declare_parameter('show_preview', False)
        self.declare_parameter('marker_spacing_x', 0.300)
        self.declare_parameter('marker_spacing_y', 0.420)
        self.show_preview = bool(self.get_parameter('show_preview').value)
        self.last_reprojection_error = None
        self.resolution_checked = False

        intrinsics_path = str(self.get_parameter('camera_intrinsics_path').value).strip()
        try:
            with Path(intrinsics_path).open(encoding='utf-8') as handle:
                intrinsics_doc = json.load(handle)
            self.camera_matrix = np.array(
                intrinsics_doc['camera_matrix'], dtype=np.float64,
            ).reshape(3, 3)
            self.dist_coeffs = np.array(
                intrinsics_doc['distortion_coefficients'], dtype=np.float64,
            ).reshape(-1)
            self.intrinsics_width = int(intrinsics_doc.get('width', 0))
            self.intrinsics_height = int(intrinsics_doc.get('height', 0))
        except (OSError, ValueError, KeyError) as exc:
            raise RuntimeError(
                f'Failed to load camera intrinsics from {intrinsics_path}: {exc}',
            ) from exc
        self.get_logger().info(
            f'Loaded intrinsics from {intrinsics_path} '
            f'({self.intrinsics_width}x{self.intrinsics_height if self.intrinsics_height else "?"})',
        )

        self.image_sub = self.create_subscription(
            Image,
            str(self.get_parameter('camera_image_topic').value),
            self.image_callback,
            10)

        self.bridge = CvBridge()
        self.calibration_done = False

        # Same annotated frames as the 'ArUco Detection' window, for headless runs.
        annotated_topic = str(self.get_parameter('extrinsics_image_topic').value).strip()
        self.annotated_pub = (
            self.create_publisher(Image, annotated_topic, 10) if annotated_topic else None
        )
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = None
        self.aruco_detector = None
        self.use_modern_aruco_api = False
        self.setup_aruco_backend()
        
        # 3x3 marker grid on the ground: x to the right, y forward, origin at marker 7.
        # Center-to-center spacing comes from calibration_params.yaml and directly
        # scales the translation (camera height), so it must match the real board.
        marker_spacing_x = float(self.get_parameter('marker_spacing_x').value)
        marker_spacing_y = float(self.get_parameter('marker_spacing_y').value)
        self.obj_points = np.array([
            [-1 , 2 , 0.0],  # Marker 0
            [0, 2, 0.0],  # Marker 1
            [1, 2, 0.0],  # Marker 2
            [-1, 1, 0.0],  # Marker 3
            [0, 1, 0.0],  # Marker 4
            [1, 1, 0.0],  # Marker 5
            [-1, 0, 0.0],  # Marker 6
            [0, 0, 0.0],  # Marker 7
            [1, 0, 0.0],  # Marker 8
        ], dtype=np.float32)

        self.obj_points[:, 0] *= marker_spacing_x
        self.obj_points[:, 1] *= marker_spacing_y
        self.get_logger().info("Objective points: " + str(self.obj_points))
        # Optional OpenCV window (needs a display); extrinsics_image_topic works headless.
        if self.show_preview:
            cv2.namedWindow('ArUco Detection', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('ArUco Detection', 640, 480)

    def write_extrinsics_file(self, rmat, tvec):
        output_path = str(self.get_parameter('camera_extrinsics_path').value).strip()
        if not output_path:
            self.get_logger().info('No camera_extrinsics_path configured. Skipping extrinsics file write.')
            return

        path = Path(output_path)
        document = {
            'rotation_matrix': np.asarray(rmat, dtype=np.float64).reshape(-1).tolist(),
            'translation': np.asarray(tvec, dtype=np.float64).reshape(-1).tolist(),
        }
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('w', encoding='utf-8') as handle:
            json.dump(document, handle, indent=2)
            handle.write('\n')
        self.get_logger().info(f'Wrote extrinsics to: {path}')

    def setup_aruco_backend(self):
        if hasattr(cv2.aruco, 'DetectorParameters') and hasattr(cv2.aruco, 'ArucoDetector'):
            self.setup_modern_aruco_backend()
        elif hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.setup_legacy_aruco_backend()
        else:
            raise RuntimeError(
                f'Unsupported OpenCV ArUco API in cv2 {cv2.__version__}: '
                'missing both DetectorParameters/ArucoDetector and DetectorParameters_create.'
            )

    def setup_modern_aruco_backend(self):
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.use_modern_aruco_api = True
        self.get_logger().info(f'Using modern ArUco API from OpenCV {cv2.__version__}')

    def setup_legacy_aruco_backend(self):
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.aruco_detector = None
        self.use_modern_aruco_api = False
        self.get_logger().info(f'Using legacy ArUco API from OpenCV {cv2.__version__}')

    def detect_markers(self, image):
        if self.use_modern_aruco_api:
            return self.aruco_detector.detectMarkers(image)

        return cv2.aruco.detectMarkers(
            image,
            self.aruco_dict,
            parameters=self.aruco_params,
        )

    def check_resolution(self, msg):
        if self.resolution_checked:
            return
        self.resolution_checked = True
        if not (self.intrinsics_width and self.intrinsics_height):
            return
        if (msg.width, msg.height) != (self.intrinsics_width, self.intrinsics_height):
            self.get_logger().warn(
                f'Camera image is {msg.width}x{msg.height} but the intrinsics were '
                f'calibrated at {self.intrinsics_width}x{self.intrinsics_height}; '
                'the extrinsics will be wrong. Recalibrate intrinsics or fix the camera mode.'
            )

    def image_callback(self, msg):
        if self.calibration_done:
            return
        self.check_resolution(msg)

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            display_image = cv_image.copy()

            corners, ids, rejected = self.detect_markers(cv_image)
            self.get_logger().info(
                f"Detected {len(ids) if ids is not None else 0} markers "
                f"(need at least {self.obj_points.shape[0] - 1} of {self.obj_points.shape[0]}).",
                throttle_duration_sec=1.0,
            )

            if ids is not None and len(ids) >= self.obj_points.shape[0] - 1:
                cv2.aruco.drawDetectedMarkers(display_image, corners, ids, (0, 255, 0))

                obj_points = []
                img_points = []
                
                for i, marker_id in enumerate(ids.flatten()):
                    if marker_id < self.obj_points.shape[0]:
                        obj_points.append(self.obj_points[marker_id])
                        img_points.append(corners[i][0].mean(axis=0))
                
                # At least 8 of the 9 known markers (all 9 is the normal case).
                if len(obj_points) >= self.obj_points.shape[0] - 1:
                    obj_points = np.array(obj_points, dtype=np.float32)
                    img_points = np.array(img_points, dtype=np.float32)
                    
                    success, rvec, tvec = cv2.solvePnP(
                        obj_points,
                        img_points,
                        self.camera_matrix,
                        self.dist_coeffs
                    )
                    
                    if success:
                        rmat, _ = cv2.Rodrigues(rvec)
                        tvec = tvec.flatten()  # Ensure tvec is 1D for printing
                        
                        # Format R and t for easy copy-paste
                        r_str = ', '.join(f"{x:.8f}" for x in rmat.flatten())
                        t_str = ', '.join(f"{x:.8f}" for x in tvec)
                        
                        self.get_logger().info('Camera extrinsics calibrated successfully!')
                        self.get_logger().info('Rotation matrix (R):')
                        self.get_logger().info(f"[{r_str}]")
                        self.get_logger().info('Translation vector (t):')
                        self.get_logger().info(f"[{t_str}]")
                        self.write_extrinsics_file(rmat, tvec)
                        self.calibration_done = True

                        self.get_logger().info('Camera height: {:.2f} meters'.format((-rmat.T @ tvec)[2]))

                        img_points_proj, _ = cv2.projectPoints(obj_points, rvec, tvec, self.camera_matrix, self.dist_coeffs)
                        img_points_proj = img_points_proj.squeeze()
                        reprojection_error = np.sqrt(np.mean(np.sum((img_points - img_points_proj)**2, axis=1)))
                        self.last_reprojection_error = reprojection_error
                        self.get_logger().info(f'Reprojection error: {reprojection_error:.4f} pixels')
                        for i, pt in enumerate(img_points_proj):
                            cv2.circle(display_image, tuple(pt.astype(int)), 5, (0, 0, 255), -1)

            if self.annotated_pub is not None:
                out = self.bridge.cv2_to_imgmsg(display_image, encoding='bgr8')
                out.header = msg.header
                self.annotated_pub.publish(out)

            if self.show_preview:
                cv2.imshow('ArUco Detection', display_image)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    raise KeyboardInterrupt

        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ExtrinsicCalibrator()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.show_preview:
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
