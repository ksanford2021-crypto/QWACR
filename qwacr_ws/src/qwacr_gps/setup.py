from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'qwacr_gps'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Create lib directory for ros2 launch compatibility
        (os.path.join('lib', package_name), []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kyle',
    maintainer_email='ksanford2021@fau.edu',
    description='GPS driver for QWACR robot with NavSatFix and ENU conversion',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_driver_node = qwacr_gps.gps_driver_node:main',
        ],
    },
)
