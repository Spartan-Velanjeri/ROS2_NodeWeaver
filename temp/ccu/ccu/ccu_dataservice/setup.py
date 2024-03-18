from os.path import abspath as abs, dirname as dir, join
from setuptools import find_packages, setup
from glob import glob

package_name = 'ccu_dataservice'

# ROS2: ament_python: Package <PACKAGE> doesn't explicitly install a marker in the package index
# ROS2: ament_python.build:Package <PACKAGE> doesn't explicitly install the 'package.xml' file

setup(
    version='2.1.0',
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['requirements.txt']),
    ],
    package_data={package_name: ['xsd/DataTypes/DataTypes-1.0/*.xsd',
                                 'xsd/NodeTree/NodeTree-2.0/*.xsd',
                                 'xsd/TaskPlan/TaskPlan-2.1/*.xsd',]},
    description='(None-ROS) Libs and core implementation for (ROS)ccu_data_services.',
    license='Copyright (c) Robert Bosch GmbH and its subsidiaries. All rights reserved.',
    package_dir={'': 'src'},
    packages=find_packages(where='src'),
    name=package_name,
    include_package_data=True,
    install_requires=(lambda: [r for r in open(join(abs(dir(__file__)), 'requirements.txt'))])(),
    classifiers=["Development Status :: 3 - Alpha"]
)
