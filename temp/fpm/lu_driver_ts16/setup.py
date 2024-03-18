from setuptools import setup

package_name = 'ts16'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rs232_driver.py']),
        ('share/' + package_name + '/config', ['config/commands.yaml']),
        ('share/' + package_name + '/config', ['config/answers.yaml']),
        ('share/' + package_name + '/config', ['config/message_fields.yaml']),
        ('share/' + package_name + '/config', ['config/readme.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='erz2lr',
    maintainer_email='michael.erz@de.bosch.com',
    description='Leica TS16 driver',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
            'console_scripts': [
                    #'talker = ts16.publisher_member_function:main',
                    'driver = ts16.driver:main',
            ],
    },    
)
