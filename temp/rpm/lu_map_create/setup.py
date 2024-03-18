from setuptools import setup

import os
from glob import glob

package_name = 'lu_map_create'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        # During installation, we need to copy the launch files
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        # Same with the RViz configuration file.
        (os.path.join('share', package_name), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'pcd_publisher_node = lu_map_create.pcd_publisher_node:main',
            'pcd_publisher_for_rokit_node = lu_map_create.pcd_publisher_for_rokit_node:main',
            'pcd_publisher_for_nav2 = lu_map_create.pcd_publisher_for_nav2:main',
            'pcd_publisher_for_rokit_simulate_lidar_node = lu_map_create.pcd_publisher_for_rokit_simulate_lidar_node:main',
        ],
    },
)
