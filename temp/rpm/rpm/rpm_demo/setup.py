#!/bin/python3
#
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.
from glob import glob

from setuptools import setup

package_name = 'rpm_demo'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/launch', glob('launch/*.launch.py')))
data_files.append(('share/' + package_name + '/config', glob('config/*.yaml')))
data_files.append(('share/' + package_name + '/rviz', glob('rviz/*.rviz')))


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sg82fe',
    maintainer_email='Georg.Schumacher@de.bosch.com',
    description='RPM Demo Package',
    license='Copyright (c) Robert Bosch GmbH. All rights reserved.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twist_message_publisher = rpm_demo.twist_message_publisher:main',
            'velocity_calculator_node = rpm_demo.velocity_calculator_node:main'
        ],
    },
)
