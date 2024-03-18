maintainer='Martin',
maintainer_email='martin.dieterle@de.bosch.com',
description='Python client server PTU-communication',
license='Apache License 2.0',

from setuptools import setup

package_name = 'ptu_comm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'service = ptu_comm.ptu_comm_service:main',
            'client = ptu_comm.ptu_comm_client:main',
        ],
    },
)
