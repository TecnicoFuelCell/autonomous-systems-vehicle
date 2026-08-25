import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from pathlib import Path

class ExtrinsicCalibrator(Node):
    def __init__(self):
        super().__init__('extrinsic_calibrator')
        self.declare_parameter('camera_extrinsics_path', '')
        
        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/zed/left_camera/image_raw',
            self.image_callback,
            10)
        
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/my/camera_info',
            self.info_callback,
            10)
        
        # Initialize variables
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.calibration_done = False
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = None
        self.aruco_detector = None
        self.use_modern_aruco_api = False
        self.setup_aruco_backend()
        
        #validation values
        dist_arucos_right = 0.300  # meters
        dist_arucos_up = 0.420  # meters #!corrigir 
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

        self.obj_points[:, 0]*= dist_arucos_right
        self.obj_points[:, 1]*= dist_arucos_up
        self.get_logger().info("Objective points: " + str(self.obj_points))
        # Initialize OpenCV window
        cv2.namedWindow('ArUco Detection', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('ArUco Detection', 640, 480)

    def write_extrinsics_file(self, rmat, tvec):
        camera_extrinsics_path = str(self.get_parameter('camera_extrinsics_path').value).strip()
        if not camera_extrinsics_path:
            self.get_logger().info('No camera_extrinsics_path configured. Skipping extrinsics file write.')
            return

        path = Path(camera_extrinsics_path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)

        r_str = ', '.join(f"{x:.8f}" for x in rmat.flatten())
        t_str = ', '.join(f"{x:.8f}" for x in tvec)
        path.write_text(f"R: [{r_str}]\nt: [{t_str}]\n", encoding='utf-8')
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

    def info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Received camera intrinsics')

    def image_callback(self, msg):
        if self.camera_matrix is None or self.calibration_done:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            display_image = cv_image.copy()

            corners, ids, rejected = self.detect_markers(cv_image)
            self.get_logger().info(f"Detected {len(ids) if ids is not None else 0} markers of {self.obj_points.shape[0]} needed.")

            if ids is not None and len(ids) >= self.obj_points.shape[0] - 1:
                cv2.aruco.drawDetectedMarkers(display_image, corners, ids, (0, 255, 0))

                obj_points = []
                img_points = []
                
                for i, marker_id in enumerate(ids.flatten()):
                    if marker_id < self.obj_points.shape[0]:
                        obj_points.append(self.obj_points[marker_id])
                        img_points.append(corners[i][0].mean(axis=0))
                
                if len(obj_points) == self.obj_points.shape[0] - 1:
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
                        self.get_logger().info(f'Reprojection error: {reprojection_error:.4f} pixels')
                        for i, pt in enumerate(img_points_proj):
                            cv2.circle(display_image, tuple(pt.astype(int)), 5, (0, 0, 255), -1)

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
    except KeyboardInterrupt:
        pass
    
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
