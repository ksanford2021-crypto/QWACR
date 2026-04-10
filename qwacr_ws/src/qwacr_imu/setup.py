from setuptools import setup

package_name = 'qwacr_imu'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kyle',
    maintainer_email='kyle@example.com',
    description='ROS 2 IMU node for SparkFun 9DoF ISM330DHCX on Raspberry Pi (publishes sensor_msgs/Imu).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sparkfun_imu_node = qwacr_imu.sparkfun_imu_node:main',
        ],
    },
)
