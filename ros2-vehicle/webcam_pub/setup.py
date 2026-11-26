from setuptools import find_packages, setup

package_name = 'webcam_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='ROS 2 package for camera streaming',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'webcam_node = webcam_pub.webcam_node:main',
        ],
    },
)
