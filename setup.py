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
    description='Perception module for RoboRacer: cone detection from LiDAR and ZED 2i camera.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_processor_node = roboracer_perception.lidar_processor:main',
            'camera_processor_node = roboracer_perception.camera_processor_node:main',
            'perception_fusion_node = roboracer_perception.perception_fusion_node:main',
        ],
    },
)
