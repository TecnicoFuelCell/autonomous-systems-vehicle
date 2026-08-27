from setuptools import find_packages, setup

package_name = 'ps4_controller_pub'

setup(
    name=package_name,
    version='0.0.1',
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
    description='ROS 2 Python package for PS4 DualShock 4 controller',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ps4_node = ps4_controller_pub.ps4_node:main',
        ],
    },
)
