import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'virtual_devices'

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
    description='Virtual Linux devices for running vehicle nodes without the hardware',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'virtual_camera_node = virtual_devices.virtual_camera_node:main',
        ],
    },
)
