from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'roboracer_estimation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RoboRacer Team',
    maintainer_email='todo@todo.com',
    description='State estimation module for RoboRacer: adaptive EKF fusing IMU, visual odometry, and wheel odometry.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'adaptive_covariance_node = roboracer_estimation.adaptive_covariance_node:main',
            'ekf_validator_node = roboracer_estimation.ekf_validator_node:main',
            'moving_obstacle_controller = roboracer_estimation.moving_obstacle_controller:main',
        ],
    },
)
