from os.path import abspath as abs, dirname as dir, join
from setuptools import find_packages, setup

package_name = 'ccu_grpc'


# ROS2: ament_python: Package <PACKAGE> doesn't explicitly install a marker in the package index
# ROS2: ament_python.build:Package <PACKAGE> doesn't explicitly install the 'package.xml' file
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name, ['requirements.txt']))

package_data = {'': ['*.pyi']}

setup(
    version='2.1.0',
    description='GENERATED CODE: CLIENT SERVER CODE, GRPC, PROTOBUF',
    license='Copyright (c) Robert Bosch GmbH and its subsidiaries. All rights reserved.',
    package_dir={'': 'src'},
    packages=find_packages(where='src'),
    name=package_name,
    data_files=data_files,
    package_data=package_data,
    include_package_data=True,
    install_requires=(lambda: [r for r in open(join(abs(dir(__file__)), 'requirements.txt'))])(),
    classifiers=["Development Status :: 3 - Alpha"],
)
