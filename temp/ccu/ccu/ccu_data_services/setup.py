# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from os import scandir
from os.path import abspath as abs, dirname as dir, basename as bn
from setuptools import find_packages, setup
from glob import glob
from ament_index_python.packages import get_package_share_directory

package_name = 'ccu_data_services'


def get_data_files():
    df = []
    df.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
    df.append(('share/' + package_name, ['package.xml']))
    df.append(('share/' + package_name, ['requirements.txt']))
    df.append(('share/' + package_name + '/launch', glob('launch/*.launch.py')))
    df.append(('share/' + package_name + '/config', glob('config/*.yaml')))
    df.append(('share/' + package_name + '/missions/1', glob('missions/1/**')))
    for m in scandir(abs(dir(__file__)) + '/missions'):
        df.append(('share/' + package_name + '/missions/' + bn(m), glob('missions/' + bn(m) + '/**')))
    return df


setup(
    entry_points={'console_scripts': ['data_service = ccu_data_services.data_service:main']},
    data_files=get_data_files(),
    version='2.1.0',
    maintainer='sg82fe',
    maintainer_email='Georg.Schumacher@de.bosch.com',
    description='TaskPlan, Roomplan loading, persistency of mission events (Hole drilled state).',
    license='Copyright (c) Robert Bosch GmbH. All rights reserved.',
    packages=find_packages(exclude=['test']),
    zip_safe=True,
    name=package_name,
    install_requires=(lambda: [r for r in open(abs(dir(__file__)) + '/requirements.txt')])(),
    tests_require=['pytest'],
)
