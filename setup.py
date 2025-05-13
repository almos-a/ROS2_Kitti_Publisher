from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'kitti_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.lua')),
        ('share/kitti_publisher/launch', ['launch/kitti_playback.launch.py']),
        ('share/kitti_publisher/launch', ['launch/kitti_raw_playback.launch.py']),
        ('share/kitti_publisher/launch', ['launch/cartographer.launch.py']),
        #('share/kitti_publisher/launch', ['launch/kitti_rviz.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='almos',
    maintainer_email='almos@gmail.com',
    description='Kitti Sequence publisher',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kitti_publisher = kitti_publisher.kitti_publisher:main',
            'kitti_raw_publisher = kitti_publisher.kitti_raw_publisher:main',
            'kitti_frame_teleop = kitti_publisher.kitti_frame_teleop:main',
        ],
    },
)
