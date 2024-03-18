from setuptools import find_packages, setup

package_name = 'bautiro_developer_functions'

setup(
    name=package_name,
    version='2.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sg82fe',
    maintainer_email='Georg.Schumacher@de.bosch.com',
    description='Developer Functions, called by HCU, via HALC',
    license='Copyright (c) Robert Bosch GmbH and its subsidiaries. All rights reserved.',
    tests_require=['pytest'],
    # entry_points={        'console_scripts': [        ],    },
)
