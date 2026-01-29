from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'qwacr_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kyle',
    maintainer_email='ksanford2021@fau.edu',
    description='Navigation stack for QWACR autonomous robot',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gps_waypoint_follower = qwacr_navigation.gps_waypoint_follower:main',
            'hardware_test = scripts.hardware_test:main',
        ],
    },
)
