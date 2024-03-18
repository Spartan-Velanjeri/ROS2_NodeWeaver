from setuptools import setup

package_name = 'marker_pose_calculator'

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
    maintainer='prr1le',
    maintainer_email='raamkishore.premkumar@de.bosch.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker_pose_service_node = marker_pose_calculator.marker_pose_service:main',
            'marker_pose_client_node = marker_pose_calculator.marker_pose_client:main',
        ],
    },
)
