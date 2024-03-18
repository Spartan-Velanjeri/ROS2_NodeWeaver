from setuptools import setup

package_name = 'leica_pose_publisher'

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
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publish_leica_pose_node = leica_pose_publisher.publish_leica_pose:main',
        ],
    },
)
