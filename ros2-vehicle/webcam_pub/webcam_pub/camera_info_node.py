import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from car_msgs.msg import CameraExtrinsics

from vehicle_calibration.calibration_io import load_camera_extrinsics, load_camera_intrinsics

EXTRINSICS_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class CameraInfoNode(Node):
    def __init__(self):
        super().__init__('camera_info_node')

        self.declare_parameter('intrinsics_path', 'calibration/camera_intrinsics.json')
        self.declare_parameter('extrinsics_path', 'calibration/camera_extrinsics.json')
        self.declare_parameter('intrinsics_topic', '/my/camera_info')
        self.declare_parameter('extrinsics_topic', '/my/camera_extrinsics')
        self.declare_parameter('extrinsics_parent_frame', 'base_link')
        self.declare_parameter('publish_intrinsics', True)
        self.declare_parameter('publish_extrinsics', True)
        self.declare_parameter('optical_frame_id', 'zed_left_camera_optical_frame')

        intrinsics_path = str(self.get_parameter('intrinsics_path').value)
        extrinsics_path = str(self.get_parameter('extrinsics_path').value)
        self.publish_intrinsics = bool(self.get_parameter('publish_intrinsics').value)
        self.publish_extrinsics = bool(self.get_parameter('publish_extrinsics').value)
        self.optical_frame_id = str(self.get_parameter('optical_frame_id').value)

        self.intrinsics_pub = None
        self.extrinsics_pub = None
        self.intrinsics_timer = None
        self.extrinsics_timer = None

        self.mtx = None
        self.dist = None
        self.width = 0
        self.height = 0
        self.rotation_matrix = None
        self.translation = None

        if self.publish_intrinsics:
            self.mtx, self.dist, self.width, self.height = load_camera_intrinsics(intrinsics_path)
            self.intrinsics_pub = self.create_publisher(
                CameraInfo,
                str(self.get_parameter('intrinsics_topic').value),
                10,
            )
            self.intrinsics_timer = self.create_timer(0.1, self.publish_camera_info)
            self.get_logger().info(f'Loaded intrinsics from {intrinsics_path}')

        if self.publish_extrinsics:
            rotation, translation = load_camera_extrinsics(extrinsics_path)
            self.rotation_matrix = rotation
            self.translation = translation
            self.extrinsics_pub = self.create_publisher(
                CameraExtrinsics,
                str(self.get_parameter('extrinsics_topic').value),
                EXTRINSICS_QOS,
            )
            self.extrinsics_timer = self.create_timer(1.0, self.publish_extrinsics_msg)
            self.publish_extrinsics_msg()
            self.get_logger().info(f'Loaded extrinsics from {extrinsics_path}')

    def publish_camera_info(self):
        if self.intrinsics_pub is None or self.mtx is None:
            return

        msg = CameraInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.optical_frame_id
        msg.width = int(self.width) if self.width else 1280
        msg.height = int(self.height) if self.height else 720
        msg.k = [
            float(self.mtx[0, 0]), 0.0, float(self.mtx[0, 2]),
            0.0, float(self.mtx[1, 1]), float(self.mtx[1, 2]),
            0.0, 0.0, 1.0,
        ]
        msg.p = [
            float(self.mtx[0, 0]), 0.0, float(self.mtx[0, 2]), 0.0,
            0.0, float(self.mtx[1, 1]), float(self.mtx[1, 2]), 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        msg.distortion_model = 'plumb_bob'
        msg.d = [float(value) for value in self.dist.reshape(-1)]
        self.intrinsics_pub.publish(msg)

    def publish_extrinsics_msg(self):
        if self.extrinsics_pub is None or self.rotation_matrix is None:
            return

        msg = CameraExtrinsics()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter('extrinsics_parent_frame').value)
        msg.rotation_matrix = [float(value) for value in self.rotation_matrix]
        msg.translation = [float(value) for value in self.translation]
        self.extrinsics_pub.publish(msg)


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
