from setuptools import setup

package_name = 'mecanum_chassis_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/chassis_driver.launch.py']),
        ('share/' + package_name + '/config', ['config/chassis_config.yaml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='4WD Mecanum Chassis Driver for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chassis_driver = mecanum_chassis_driver.chassis_driver:main',
        ],
    },
)