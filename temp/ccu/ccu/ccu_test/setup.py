from setuptools import find_packages, setup
from glob import glob
package_name = 'ccu_test'

setup(
    name=package_name,
    entry_points={'console_scripts': ['ccu_start = ccu_test.backend_ccu_start:main',
                                      'fpm_skill = ccu_test.backend_fpm_skill:main',
                                      'hu_move   = ccu_test.backend_hu_move_relative:main',]},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    version='2.1.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sg82fe',
    maintainer_email='Georg.Schumacher@de.bosch.com',
    description='Contains Fake Backends and launch files for testing.',
    license='Copyright (c) Robert Bosch GmbH. All rights reserved.',
    tests_require=['pytest'],
)
