"""Setup description."""

from setuptools import setup

PACKAGE_NAME = 'bautiro_launch'

setup(
    name=PACKAGE_NAME,
    version='0.0.0',
    packages=[PACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'launch', 'launch_ros'],
    zip_safe=True,
    maintainer='Ingo Lütkebohle',
    maintainer_email='ingo.luetkebohle@de.bosch.com',
    description='Utilities for Bautiro Launch',
    license='Robert Bosch proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'check_sim_time_node = bautiro_launch.check_sim_time:run',
            'check_lifecycle_managers_node = bautiro_launch.check_lifecycle_managers:run'
        ],
    },
)
