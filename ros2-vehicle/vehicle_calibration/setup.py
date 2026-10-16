import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'vehicle_calibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='On-vehicle and offline calibration tools',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'calibrate_extrinsics = vehicle_calibration.calibrate_extrinsics:main',
            'calibrate_intrinsics = vehicle_calibration.calibrate_intrinsics:main',
            'calibrate_mag = vehicle_calibration.calibrate_mag:main',
            'create_calibration_nparray = vehicle_calibration.create_calibration_nparray:main',
            # Deprecated names (removed in a future release).
            'calibrate_aruco = vehicle_calibration.calibrate_extrinsics:main',
            'intrinsics_node_recorder = vehicle_calibration.calibrate_intrinsics:run_ros_node',
            'camera_intrinsics_calibrator = vehicle_calibration.calibrate_intrinsics:main_cli',
            'calibrate_camera = vehicle_calibration.calibrate_intrinsics:legacy_calibrate_camera',
            'camera_to_video = vehicle_calibration.calibrate_intrinsics:legacy_camera_to_video',
            'get_video_frame_size = vehicle_calibration.calibrate_intrinsics:legacy_get_video_frame_size',
            'record_intrinsics_video = vehicle_calibration.calibrate_intrinsics:legacy_record_intrinsics_video',
        ],
    },
)
