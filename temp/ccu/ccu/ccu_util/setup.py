from os.path import abspath as abs, dirname as dir, join
from setuptools import find_packages, setup

package_name = 'ccu_util'

# ROS2: ament_python: Package <PACKAGE> doesn't explicitly install a marker in the package index
# ROS2: ament_python.build:Package <PACKAGE> doesn't explicitly install the 'package.xml' file
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name, ['requirements.txt']),
]

setup(
    version='2.1.0',
    description='Utilities for HALC and DataService.',
    license='Copyright (c) Robert Bosch GmbH and its subsidiaries. All rights reserved.',
    package_dir={'': 'src'},
    packages=find_packages(where='src'),
    name=package_name,
    data_files=data_files,
    include_package_data=True,
    install_requires=(lambda: [r for r in open(join(abs(dir(__file__)), 'requirements.txt'))])(),
    classifiers=["Development Status :: 3 - Alpha"]
)
