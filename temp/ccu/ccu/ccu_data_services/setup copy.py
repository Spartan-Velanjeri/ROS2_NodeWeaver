# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from os import scandir
from os.path import abspath as abs, dirname as dir, basename as bn
from setuptools import find_packages, setup
from glob import glob

package_name = 'ccu_data_services'
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name, ['requirements.txt']),
    ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ('share/' + package_name + '/config', glob('config/*.yaml')),
    ('share/' + package_name + '/missions/1', glob('missions/1/*.*')),
]
for m in scandir(dir(__file__) + '/missions'):
  print(bn(m))
  the_glob = 'missions/' + bn(m) + '/**'
  print(the_glob)
  data_files.append(

    ('share/' + package_name + '/missions/' + bn(m), glob(the_glob))
    )


print(data_files)
print(data_files)
print(data_files)
