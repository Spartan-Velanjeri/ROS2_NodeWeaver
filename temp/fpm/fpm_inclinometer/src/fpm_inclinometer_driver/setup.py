import os
from glob import glob

from setuptools import setup

"""
For launch files:
add of 
import os
from glob import glob
and 
glob('launch/*.launch.py')
Based on example from
https://roboticsbackend.com/ros2-launch-file-example/
"""

package_name = 'fpm_inclinometer_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']), 
        # Include all launch files.
        (os.path.join('share', package_name, "launch"), glob('launch/*launch.[pxy][yma]*')),
        # Include rviz configuration.
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='die2si',
    maintainer_email='die2si@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'digipas_driver_node = fpm_inclinometer_driver.digipas_driver_node:main',
            'inclinometer_transform_publisher_node = fpm_inclinometer_driver.inclinometer_transform_publisher_node:main',
        ],
    },
)
