import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from car_msgs.msg import CameraExtrinsics
import numpy as np
import yaml

EXTRINSICS_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)

class CameraInfoNode(Node):
    def __init__(self):
        super().__init__('camera_info_node')

        self.param_names = self._declare_params()
        self.publish_intrinsics_enabled = self.get_parameter('publish_intrinsics').value
        self.publish_extrinsics_enabled = self.get_parameter('publish_extrinsics').value

        # publisher for intrinsics and extrinsics
        self.publisher_ = None
        self.extrinsics_pub_ = None
        if self.publish_intrinsics_enabled:
            self.publisher_ = self.create_publisher(CameraInfo, self.get_parameter('intrinsics_topic').value, 10)
        if self.publish_extrinsics_enabled:
            self.extrinsics_pub_ = self.create_publisher(
                CameraExtrinsics,
                self.get_parameter('extrinsics_topic').value,
                EXTRINSICS_QOS,
            )

        # Load calibration data from .npz file
        if self.publish_intrinsics_enabled:
            self.load_calibration_data(self.get_parameter('intrinsics_data').value)
        if self.publish_extrinsics_enabled:
            self.load_extrinsics_data(self.get_parameter('extrinsics_data').value)

        # timers to publish messages periodically
        self.timer = None
        self.extrinsics_timer = None
        if self.publish_intrinsics_enabled:
            self.timer = self.create_timer(0.1, self.publish_camera_info) # every 0.1 seconds
        if self.publish_extrinsics_enabled:
            self.extrinsics_timer = self.create_timer(1, self.publish_extrinsics_info) # every 1 second
            self.publish_extrinsics_info()

        # image dimensions (double check with calibration)
        self.width = 1280
        self.height = 720
        
        if self.publish_intrinsics_enabled:
            self.get_logger().info("Using intrinsics data: " + self.get_parameter('intrinsics_data').value)
        if self.publish_extrinsics_enabled:
            self.get_logger().info("Using extrinsics data: " + self.get_parameter('extrinsics_data').value)
            self.get_logger().info("Publishing extrinsics on: " + self.get_parameter('extrinsics_topic').value)
        self.get_logger().info("[WARNING] CHECK IF THIS NP ARRAY IS WHAT YOU WANT")
        self.get_logger().info("Width: " + str(self.width) + ", Height: " + str(self.height))
        self.get_logger().debug("Node parameters declared:\n"
            + " | ".join(f"{name}: {self.get_parameter(name).value}" for name in self.param_names)
        )

    def load_calibration_data(self, filepath):
        # Load the .npz file
        with np.load(filepath) as data:
            mtx = data['mtx']  # Camera matrix
            dist = data['dist']  # Distortion coefficients

        # Extract camera parameters from mtx
        self.fx = mtx[0, 0]  # fx
        self.cx = mtx[0, 2]  # cx
        self.fy = mtx[1, 1]  # fy
        self.cy = mtx[1, 2]  # cy

        # Store distortion coefficients (assuming 5 values for plumb_bob)
        self.dist = dist.flatten().tolist()  # Convert to list for ROS message

    def load_extrinsics_data(self, filepath):
        with open(filepath, 'r') as f:
            extrinsics = yaml.safe_load(f)

        rotation_matrix = extrinsics.get('R')
        translation = extrinsics.get('t')

        if rotation_matrix is None or len(rotation_matrix) != 9:
            raise ValueError(f"{filepath!r} must contain an R list with 9 values")
        if translation is None or len(translation) != 3:
            raise ValueError(f"{filepath!r} must contain a t list with 3 values")

        self.rotation_matrix = [float(value) for value in rotation_matrix]
        self.translation = [float(value) for value in translation]
        self.get_logger().info(
            "Loaded camera extrinsics:\n"
            f"R=\n{np.array(self.rotation_matrix, dtype=np.float64).reshape(3, 3)}\n"
            f"t=\n{np.array(self.translation, dtype=np.float64).reshape(3, 1)}"
        )

    def publish_camera_info(self):
        if self.publisher_ is None:
            return

        # Create CameraInfo message
        msg = CameraInfo()
        
        # Set header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'zed_left_camera_optical_frame'  # Adjust as needed

        # Set image dimensions
        msg.height = self.height
        msg.width = self.width

        # Set camera matrix (K)
        msg.k = [
            self.fx, 0.0, self.cx,
            0.0, self.fy, self.cy,
            0.0, 0.0, 1.0
        ]

        # Set projection matrix (P) - assuming monocular (no baseline)
        msg.p = [
            self.fx, 0.0, self.cx, 0.0,  # tx = 0
            0.0, self.fy, self.cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        # Set distortion parameters
        msg.distortion_model = 'plumb_bob'
        msg.d = self.dist  # k1, k2, p1, p2, k3

        # Publish the message
        self.publisher_.publish(msg)
        #self.get_logger().info('Published CameraInfo')

    def publish_extrinsics_info(self):
        if self.extrinsics_pub_ is None:
            return

        msg = CameraExtrinsics()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('extrinsics_parent_frame').value

        msg.rotation_matrix = self.rotation_matrix
        msg.translation = self.translation

        self.extrinsics_pub_.publish(msg)

    def _declare_params(self):
        # function to declare defaults for params, will be overriden by car_params.yaml
        # when running from launchfile
        self.declare_parameters(
            namespace='',
            parameters=[
                ('intrinsics_topic', 'my/camera_info'),
                ('intrinsics_data', 'config/calibration_elgato_1280_720.npz'),
                ('extrinsics_topic', 'my/camera_extrinsics'),
                ('extrinsics_data', 'config/camera_extrinsics.yaml'),
                ('extrinsics_parent_frame', 'base_link'),
                ('extrinsics_child_frame', 'zed_left_camera_optical_frame'),
                ('publish_intrinsics', True),
                ('publish_extrinsics', True),
            ]
        )
        return [
            'intrinsics_topic',
            'intrinsics_data',
            'extrinsics_topic',
            'extrinsics_data',
            'extrinsics_parent_frame',
            'extrinsics_child_frame',
            'publish_intrinsics',
            'publish_extrinsics',
        ]

def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
