from setuptools import find_packages, setup

package_name = 'roboracer_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RoboRacer Team',
    maintainer_email='todo@todo.com',
    description='Perception module for RoboRacer: solid-wall track with LIDAR PCA + ZED 2i color detection.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_opponent_detector = roboracer_perception.wall_opponent_detector:main',
            'camera_processor_node = roboracer_perception.camera_processor:main',
            'sim_camera_walls = roboracer_perception.sim_camera_walls:main',
            'moving_obstacle_controller = roboracer_perception.moving_obstacle_controller:main',
            'solid_wall_visualizer = roboracer_perception.solid_wall_visualizer:main',
            'solid_wall_scan_highlighter = roboracer_perception.solid_wall_scan_highlighter:main',
            'teleop_key = roboracer_perception.teleop_key:main',
        ],
    },
)
