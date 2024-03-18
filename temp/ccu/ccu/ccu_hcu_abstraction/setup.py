from glob import glob
from setuptools import setup, find_packages

package_name = 'ccu_hcu_abstraction'

setup(
    name=package_name,
    version='2.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sg82fe',
    maintainer_email='Georg.Schumacher@de.bosch.com',
    description='Central Control Unit CCU-HCU-Abstraction',
    license='Copyright (c) Robert Bosch GmbH. All rights reserved.',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['halc = ccu_hcu_abstraction.halc:main']},
)
