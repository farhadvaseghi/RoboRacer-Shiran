import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'roboracer_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shiran RoboRacer',
    maintainer_email='mohammadsadeghshoushtari@gmail.com',
    description='ZED depth -> LaserScan of standing obstacles for the Nav2 costmap.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_to_scan = roboracer_camera.depth_to_scan_node:main',
            'opponent_detector = roboracer_camera.opponent_detector:main',
            'map_odom_relay = roboracer_camera.map_odom_relay:main',
            'person_detector = roboracer_camera.person_detector:main',
            'emergency_brake = roboracer_camera.emergency_brake:main',
            'sim_person_publisher = roboracer_camera.sim_person_publisher:main',
            'sim_test_drive = roboracer_camera.sim_test_drive:main',
            'zed_ground_calibration = roboracer_camera.zed_ground_calibration:main',
        ],
    },
)
